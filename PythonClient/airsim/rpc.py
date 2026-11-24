# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

"""MessagePack-RPC client transport without Tornado or a process-global event loop."""

from concurrent.futures import Future as _Future, TimeoutError as _FutureTimeoutError
import errno
import math
import select
import socket
import threading
import time
import weakref

import msgpack


# getaddrinfo cannot be cancelled portably. Cap outstanding daemon lookups even
# across clients; callers wait for a slot/result only within their own deadline.
_RESOLVER_SLOTS = threading.BoundedSemaphore(4)


def _resolve_address(address, future, slots):
    try:
        future.set_result(socket.getaddrinfo(*address, 0, socket.SOCK_STREAM))
    except Exception as error:
        future.set_exception(error)
    finally:
        slots.release()


class RPCError(Exception):
    def __init__(self, message):
        super().__init__(message)
        self.data = message


class RPCTimeoutError(RPCError, TimeoutError):
    pass


class RPCTransportError(RPCError, ConnectionError):
    pass


class Future:
    """Compatible get()/join() facade; RPC timeout does not cancel server work."""

    def __init__(self, deadline):
        self._future = _Future()
        self._deadline = deadline
        self._result_handler = None
        self._error_handler = None

    def get(self, timeout=None):
        try:
            result = self._future.result(timeout=timeout)
        except RPCError as error:
            if self._error_handler is not None:
                self._error_handler(error)
                return None
            raise
        except _FutureTimeoutError as error:
            raise RPCTimeoutError('Timed out waiting for RPC result') from error
        if self._result_handler is not None:
            self._result_handler(result)
            return None
        return result

    def join(self, timeout=None):
        # Legacy join waits but does not surface a remote error; get does.
        try:
            self._future.result(timeout=timeout)
        except RPCError:
            pass
        except _FutureTimeoutError as error:
            raise RPCTimeoutError('Timed out waiting for RPC completion') from error

    @property
    def result(self):
        if not self._future.done() or self._future.exception() is not None:
            return None
        return self._future.result()

    @property
    def error(self):
        return self._future.exception() if self._future.done() else None

    def attach_callback(self, callback):
        # A callback may issue a blocking RPC; it must not block the receive loop.
        self._future.add_done_callback(lambda unused: threading.Thread(
            target=callback, args=(self,), name='airsim-rpc-callback', daemon=True).start())

    def attach_result_handler(self, handler):
        self._result_handler = handler

    def attach_error_handler(self, handler):
        self._error_handler = handler


def _pack_default(value):
    method = getattr(value, 'to_msgpack', None)
    if method is None:
        raise TypeError('Cannot serialize ' + type(value).__name__)
    return method()


class Client:
    def __init__(self, host, port, timeout=3600):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('RPC timeout must be finite and positive')
        self._address = (host, int(port))
        self._timeout = float(timeout)
        self._lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._pending = {}
        self._next_id = 0
        self._socket = None
        self._connection_generation = 0
        self._connecting_socket = None
        self._resolution = None
        self._reader = None
        self._closed = False

    @property
    def connection_generation(self):
        """Local physical-connection identity; increases on every successful connect."""
        with self._lock:
            return self._connection_generation

    def call(self, method, *args):
        return self.call_async(method, *args).get()

    def call_async(self, method, *args):
        deadline = time.monotonic() + self._timeout
        future = Future(deadline)
        with self._lock:
            if self._closed:
                raise RPCTransportError('RPC client is closed')
            message_id = self._next_id
            while message_id in self._pending:
                message_id = (message_id + 1) % (2 ** 32)
            self._next_id = (message_id + 1) % (2 ** 32)
        payload = msgpack.packb([0, message_id, method, args], default=_pack_default, use_bin_type=True)
        connection = None
        acquired = registered = False
        try:
            self._acquire_until(self._send_lock, deadline, 'send')
            acquired = True
            with self._lock:
                if self._closed:
                    raise RPCTransportError('RPC client is closed')
                connection = self._socket
                self._pending[message_id] = future
                registered = True
            if connection is None:
                connection = self._connect(deadline)
            view = memoryview(payload)
            while view:
                remaining = self._remaining(deadline, 'send')
                if future._future.done():
                    raise RPCTransportError('RPC connection ended before send completed')
                unused_read, writable, unused_error = select.select([], [connection], [], min(0.1, remaining))
                if not writable:
                    continue
                try:
                    sent = connection.send(view)
                except BlockingIOError:
                    continue
                if sent == 0:
                    raise RPCTransportError('RPC connection closed during send')
                view = view[sent:]
        except (OSError, ValueError, RPCError) as error:
            if not isinstance(error, RPCError):
                error = RPCTimeoutError(str(error)) if isinstance(error, TimeoutError) else RPCTransportError(str(error))
            if connection is not None:
                self._disconnect(connection, error)
            with self._lock:
                pending = self._pending.pop(message_id, None)
            if pending is not None:
                pending._future.set_exception(error)
            elif not registered:
                future._future.set_exception(error)
            # Once registered, whoever removes it from _pending owns completion.
        finally:
            if acquired:
                self._send_lock.release()
        return future

    def _remaining(self, deadline, phase):
        with self._lock:
            if self._closed:
                raise RPCTransportError('RPC client is closed')
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RPCTimeoutError('RPC ' + phase + ' deadline exceeded')
        return remaining

    def _acquire_until(self, lock, deadline, phase):
        while not lock.acquire(timeout=min(0.1, self._remaining(deadline, phase))):
            pass

    def _resolve(self, deadline):
        host, port = self._address
        for family in (socket.AF_INET, socket.AF_INET6):
            try:
                socket.inet_pton(family, host)
            except OSError:
                continue
            address = (host, port) if family == socket.AF_INET else (host, port, 0, 0)
            return [(family, socket.SOCK_STREAM, socket.IPPROTO_TCP, '', address)]

        if self._resolution is None:
            slots = _RESOLVER_SLOTS
            self._acquire_until(slots, deadline, 'connection')
            try:
                self._remaining(deadline, 'connection')
                resolution = _Future()
                threading.Thread(target=_resolve_address, args=(self._address, resolution, slots),
                                 name='airsim-rpc-resolver', daemon=True).start()
                self._resolution = resolution
            except BaseException:
                slots.release()
                raise
        resolution = self._resolution
        try:
            while True:
                remaining = self._remaining(deadline, 'connection')
                if resolution.done():
                    return resolution.result()
                try:
                    resolution.result(timeout=min(0.1, remaining))
                except _FutureTimeoutError:
                    pass
        finally:
            # Reuse a still-running lookup on retry rather than spawning more.
            if resolution.done():
                self._resolution = None

    def _connect(self, deadline):
        addresses = self._resolve(deadline)
        last_error = None
        for family, kind, protocol, unused_name, address in addresses:
            self._remaining(deadline, 'connection')
            connection = None
            connected = False
            try:
                connection = socket.socket(family, kind, protocol)
                connection.setblocking(False)
                with self._lock:
                    if self._closed:
                        raise RPCTransportError('RPC client is closed')
                    self._connecting_socket = connection
                code = connection.connect_ex(address)
                while code in (errno.EINPROGRESS, errno.EWOULDBLOCK, errno.EALREADY, errno.EINTR):
                    remaining = self._remaining(deadline, 'connection')
                    unused_read, writable, exceptional = select.select(
                        [], [connection], [connection], min(0.1, remaining))
                    if writable or exceptional:
                        code = connection.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
                if code not in (0, errno.EISCONN):
                    raise OSError(code, errno.errorcode.get(code, 'Connection failed'))
                self._remaining(deadline, 'connection')
                with self._lock:
                    if self._closed:
                        raise RPCTransportError('RPC client is closed')
                    self._socket = connection
                    self._connection_generation += 1
                    self._connecting_socket = None
                    connected = True
                    self._reader = threading.Thread(target=Client._receive, args=(weakref.ref(self), connection),
                                                    name='airsim-rpc', daemon=True)
                    self._reader.start()
                return connection
            except RPCError:
                raise
            except OSError as error:
                last_error = error
            finally:
                if not connected and connection is not None:
                    with self._lock:
                        if self._connecting_socket is connection:
                            self._connecting_socket = None
                    connection.close()
        self._remaining(deadline, 'connection')
        raise last_error or RPCTransportError('No addresses found for RPC server')

    def _expire(self):
        now = time.monotonic()
        with self._lock:
            expired = [key for key, future in self._pending.items() if future._deadline <= now]
            futures = [self._pending.pop(key) for key in expired]
        for future in futures:
            future._future.set_exception(RPCTimeoutError('RPC request timed out; server work may still be running'))

    def _dispatch(self, response):
        if not isinstance(response, (list, tuple)) or len(response) != 4 or response[0] != 1 or type(response[1]) is not int:
            raise RPCTransportError('Invalid MessagePack-RPC response')
        with self._lock:
            future = self._pending.pop(response[1], None)
        if future is None:
            return
        if future._deadline <= time.monotonic():
            future._future.set_exception(RPCTimeoutError('RPC response arrived after its deadline'))
        elif response[2] is not None:
            future._future.set_exception(RPCError(response[2]))
        else:
            future._future.set_result(response[3])

    @staticmethod
    def _receive(owner_ref, connection):
        # Legacy servers may encode binary image strings as STR rather than BIN.
        unpacker = msgpack.Unpacker(raw=False, unicode_errors='surrogateescape', strict_map_key=False,
                                    max_buffer_size=256 * 1024 * 1024)
        try:
            while True:
                owner = owner_ref()
                if owner is None:
                    connection.close()
                    return
                with owner._lock:
                    active = not owner._closed and owner._socket is connection
                if not active:
                    return
                owner._expire()
                del owner
                readable, unused_write, unused_error = select.select([connection], [], [], 0.1)
                if not readable:
                    continue
                try:
                    data = connection.recv(65536)
                except BlockingIOError:
                    continue
                if not data:
                    raise RPCTransportError('RPC server closed the connection; in-flight calls are not replayed')
                unpacker.feed(data)
                for response in unpacker:
                    owner = owner_ref()
                    if owner is None:
                        connection.close()
                        return
                    owner._dispatch(response)
                    del owner
        except Exception as error:
            owner = owner_ref()
            if owner is not None:
                owner._disconnect(connection, error if isinstance(error, RPCError) else RPCTransportError(str(error)))
            else:
                connection.close()

    def _disconnect(self, connection, error):
        with self._lock:
            if self._socket is not connection:
                return
            self._socket = None
            pending, self._pending = self._pending, {}
        try:
            connection.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        connection.close()
        for future in pending.values():
            future._future.set_exception(error)

    def close(self):
        with self._lock:
            self._closed = True
            connection = self._socket
            connecting, self._connecting_socket = self._connecting_socket, None
            reader = self._reader
            pending, self._pending = self._pending, {}
        if connecting is not None:
            connecting.close()
        if connection is not None:
            self._disconnect(connection, RPCTransportError('RPC client closed'))
        for future in pending.values():
            future._future.set_exception(RPCTransportError('RPC client closed'))
        if reader is not None and reader is not threading.current_thread():
            reader.join(timeout=1)

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
