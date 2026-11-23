import concurrent.futures
import errno
import gc
from pathlib import Path
import socket
import socketserver
import sys
import threading
import time
import unittest
from unittest import mock
import weakref

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import airsim
import airsim.rpc as rpc
import msgpack
from airsim.rpc import Client, Future, RPCError, RPCTimeoutError, RPCTransportError


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


class Handler(socketserver.BaseRequestHandler):
    def setup(self):
        self.send_lock = threading.Lock()

    def send(self, message_id, value, error=None, delay=0, fragment=False, legacy=False):
        def reply():
            if delay:
                time.sleep(delay)
            data = msgpack.packb([1, message_id, error, value], use_bin_type=not legacy)
            try:
                with self.send_lock:
                    if fragment:
                        for byte in data:
                            self.request.sendall(bytes([byte]))
                    else:
                        self.request.sendall(data)
            except OSError:
                pass
        if delay:
            threading.Thread(target=reply, daemon=True).start()
        else:
            reply()

    def handle(self):
        parser = msgpack.Unpacker(raw=False)
        while True:
            try:
                data = self.request.recv(65536)
            except OSError:
                return
            if not data:
                return
            parser.feed(data)
            for kind, identifier, method, params in parser:
                if kind != 0:
                    return
                if method == 'ping':
                    self.send(identifier, True)
                elif method == 'echo':
                    self.send(identifier, params[0])
                elif method == 'delayed':
                    self.send(identifier, params[1], delay=params[0])
                elif method == 'error':
                    self.send(identifier, None, error={'message': 'remote failure', 'code': 42})
                elif method == 'fragmented':
                    self.send(identifier, params[0], fragment=True)
                elif method == 'disconnect':
                    return
                elif method == 'invalid':
                    self.request.sendall(msgpack.packb([9, identifier]))
                elif method == 'simGetImage':
                    self.send(identifier, b'\x89PNG\xff\x00', legacy=True)
                elif method == 'getMultirotorState':
                    self.send(identifier, {'landed_state': 0, 'timestamp': 123,
                                           'kinematics_estimated': {'position': {'x_val': 1.5, 'y_val': 2.5, 'z_val': -3.0}}})
                else:
                    self.send(identifier, None, error='unknown method')


class TransportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = Server(('127.0.0.1', 0), Handler)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.port = cls.server.server_address[1]

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join(timeout=2)

    def setUp(self):
        self.client = Client('127.0.0.1', self.port, timeout=2)

    def tearDown(self):
        self.client.close()

    def test_sync_roundtrip_and_binary_data(self):
        value = {'text': '无人机', 'bytes': b'\x00\xff\x80', 'float': 3.25, 'values': [True, False, None, -7]}
        self.assertEqual(self.client.call('echo', value), value)

    def test_custom_airsim_types(self):
        self.assertEqual(self.client.call('echo', airsim.Vector3r(1, 2, 3)), {'x_val': 1, 'y_val': 2, 'z_val': 3})
        with self.assertRaises(TypeError):
            self.client.call('echo', object())
        self.assertTrue(self.client.call('ping'))

    def test_async_futures_match_out_of_order_responses(self):
        first = self.client.call_async('delayed', 0.15, 'first')
        second = self.client.call_async('delayed', 0.01, 'second')
        self.assertEqual(second.get(), 'second')
        self.assertIsNone(first.join())
        self.assertEqual(first.get(), 'first')
        self.assertEqual(first.result, 'first')
        self.assertIsNone(first.error)

    def test_multiple_calling_threads(self):
        with concurrent.futures.ThreadPoolExecutor(max_workers=8) as executor:
            results = list(executor.map(lambda n: self.client.call('echo', n), range(40)))
        self.assertEqual(results, list(range(40)))

    def test_fragmented_tcp_response(self):
        value = {'blob': bytes(range(256)), 'nested': {'value': 7}}
        self.assertEqual(self.client.call('fragmented', value), value)

    def test_error_propagation_and_legacy_join(self):
        future = self.client.call_async('error')
        self.assertIsNone(future.join())
        with self.assertRaises(RPCError) as error:
            future.get()
        self.assertEqual(error.exception.data['code'], 42)
        self.assertIsInstance(future.error, RPCError)
        self.assertIsNone(future.result)
        handled = []
        future.attach_error_handler(lambda value: handled.append(value.data['code']) or 42)
        self.assertIsNone(future.get())
        self.assertEqual(handled, [42])

    def test_request_timeout_without_waiting_and_late_response(self):
        self.client.close()
        self.client = Client('127.0.0.1', self.port, timeout=0.05)
        future = self.client.call_async('delayed', 0.2, 'late')
        time.sleep(0.15)
        with self.assertRaises(RPCTimeoutError):
            future.get()
        time.sleep(0.1)
        self.assertTrue(self.client.call('ping'))

    def test_wait_timeout_does_not_replay_or_cancel(self):
        future = self.client.call_async('delayed', 0.1, 'result')
        with self.assertRaises(RPCTimeoutError):
            future.get(timeout=0.001)
        self.assertEqual(future.get(), 'result')

    def test_disconnect_fails_pending_and_next_call_reconnects(self):
        pending = self.client.call_async('delayed', 0.5, 'not replayed')
        with self.assertRaises(RPCTransportError):
            self.client.call('disconnect')
        with self.assertRaises(RPCTransportError):
            pending.get()
        self.assertTrue(self.client.call('ping'))

    def test_close_releases_pending_and_is_idempotent(self):
        pending = self.client.call_async('delayed', 1, 'not replayed')
        self.client.close()
        self.client.close()
        with self.assertRaises(RPCTransportError):
            pending.get()
        with self.assertRaises(RPCTransportError):
            self.client.call('ping')

    def test_invalid_response_fails_closed_then_reconnects(self):
        with self.assertRaises(RPCTransportError):
            self.client.call('invalid')
        self.assertTrue(self.client.call('ping'))

    def test_callback_can_issue_another_rpc(self):
        completed = threading.Event()
        values = []
        future = self.client.call_async('echo', 9)
        def callback(result):
            values.append((result.get(), self.client.call('ping')))
            completed.set()
        future.attach_callback(callback)
        self.assertTrue(completed.wait(3))
        self.assertEqual(values, [(9, True)])

    def test_unused_client_is_collectable(self):
        client = Client('127.0.0.1', self.port, timeout=1)
        self.assertTrue(client.call('ping'))
        reference = weakref.ref(client)
        del client
        deadline = time.monotonic() + 2
        while reference() is not None and time.monotonic() < deadline:
            gc.collect()
            time.sleep(0.05)
        self.assertIsNone(reference())

    def test_public_client_and_legacy_binary_image(self):
        with airsim.MultirotorClient('127.0.0.1', self.port, timeout_value=2) as client:
            self.assertTrue(client.ping())
            state = client.getMultirotorState()
            self.assertEqual(state.kinematics_estimated.position.z_val, -3)
            self.assertEqual(state.landed_state, airsim.LandedState.Landed)
            self.assertEqual(client.simGetImage('0', airsim.ImageType.Scene), b'\x89PNG\xff\x00')

    def test_binary_numpy_helpers_return_mutable_arrays(self):
        values = airsim.string_to_uint8_array(b'\x00\xff')
        values[0] = 7
        self.assertEqual(values.tolist(), [7, 255])
        import struct
        self.assertEqual(airsim.string_to_float_array(struct.pack('ff', 1.5, 2.5)).tolist(), [1.5, 2.5])

    def test_invalid_timeout_rejected(self):
        for timeout in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                Client('127.0.0.1', self.port, timeout=timeout)


class ObservedLock:
    """Expose actual contention without timing sleeps in cancellation tests."""

    def __init__(self, held=False):
        self.lock = threading.Lock()
        self.contended = threading.Event()
        if held:
            self.lock.acquire()

    def acquire(self, timeout):
        if self.lock.locked():
            self.contended.set()
        return self.lock.acquire(timeout=timeout)

    def release(self):
        self.lock.release()


class ConnectionDeadlineTests(unittest.TestCase):
    """No network: fake sockets/clock and gated DNS exercise establishment."""

    def setUp(self):
        self.clients = []
        self.workers = []
        self.lookups = []
        self.release_dns = threading.Event()
        self.lookup_started = threading.Event()
        self.addresses = [(socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_TCP, '',
                           ('192.0.2.' + str(index), 41451)) for index in range(1, 4)]
        self.dns = self.patch(rpc.socket, 'getaddrinfo', side_effect=AssertionError('Unexpected DNS'))
        self.sockets = self.patch(rpc.socket, 'socket', side_effect=AssertionError('Unexpected socket'))
        self.patch(rpc.socket, 'create_connection', side_effect=AssertionError('Blocking connect used'))
        self.select = self.patch(rpc.select, 'select', side_effect=lambda r, w, e, timeout: ([], w, []))
        self.patch(Client, '_receive')

    def tearDown(self):
        self.release_dns.set()
        for client in self.clients:
            client.close()
        for worker in self.workers + self.lookups:
            worker.join(timeout=1)
            self.assertFalse(worker.is_alive(), 'Test left a worker running')

    def patch(self, target, name, **kwargs):
        patcher = mock.patch.object(target, name, **kwargs)
        result = patcher.start()
        self.addCleanup(patcher.stop)
        return result

    def client(self, host='rpc.test', timeout=60):
        client = Client(host, 41451, timeout=timeout)
        self.clients.append(client)
        return client

    def run_background(self, function, *args):
        result = concurrent.futures.Future()

        def run():
            try:
                result.set_result(function(*args))
            except BaseException as error:
                result.set_exception(error)

        worker = threading.Thread(target=run, daemon=True)
        self.workers.append(worker)
        worker.start()
        return result

    def block_dns(self, expected=1):
        def resolve(*args):
            self.lookups.append(threading.current_thread())
            if len(self.lookups) >= expected:
                self.lookup_started.set()
            if not self.release_dns.wait(5):
                raise AssertionError('Test did not release DNS')
            return self.addresses

        self.dns.side_effect = resolve

    def fake_socket(self, code=errno.EINPROGRESS, error=0):
        connection = mock.Mock(spec=['setblocking', 'connect_ex', 'getsockopt', 'send', 'shutdown', 'close'])
        connection.closed = threading.Event()
        connection.close.side_effect = connection.closed.set
        connection.connect_ex.return_value = code
        connection.getsockopt.return_value = error
        connection.send.side_effect = lambda data: len(data)
        return connection

    def assert_transport_failure(self, caller):
        future = caller.result(timeout=1)
        with self.assertRaises(RPCTransportError):
            future.get(timeout=1)
        return future

    def test_numeric_ipv4_and_ipv6_bypass_dns(self):
        for host, family, address in [('127.0.0.1', socket.AF_INET, ('127.0.0.1', 41451)),
                                      ('::1', socket.AF_INET6, ('::1', 41451, 0, 0))]:
            with self.subTest(host=host):
                client = self.client(host)
                connection = self.fake_socket(code=0)
                self.sockets.side_effect = None
                self.sockets.return_value = connection
                future = client.call_async('echo', 7)
                self.sockets.assert_called_with(family, socket.SOCK_STREAM, socket.IPPROTO_TCP)
                connection.connect_ex.assert_called_once_with(address)
                connection.setblocking.assert_called_once_with(False)
                self.assertEqual(msgpack.unpackb(bytes(connection.send.call_args.args[0])), [0, 0, 'echo', [7]])
                client._dispatch([1, 0, None, 7])
                self.assertEqual(future.get(), 7)
                client.close()
        self.dns.assert_not_called()

    def test_slow_dns_obeys_deadline_and_retries_share_lookup(self):
        self.block_dns()
        client = self.client(timeout=0.04)
        first = self.run_background(client.call_async, 'ping')
        self.assertTrue(self.lookup_started.wait(1))
        for caller in (first, self.run_background(client.call_async, 'ping')):
            future = caller.result(timeout=1)
            with self.assertRaises(RPCTimeoutError):
                future.get()
        # Retry after both the DNS and send-lock deadlines have expired.
        future = self.run_background(client.call_async, 'ping').result(timeout=1)
        with self.assertRaises(RPCTimeoutError):
            future.get()
        self.assertFalse(self.release_dns.is_set())
        self.assertEqual(len(self.lookups), 1)
        self.assertTrue(self.lookups[0].daemon)
        self.dns.assert_called_once()
        self.sockets.assert_not_called()
        # A late DNS result can serve a later call, but expired calls are not sent.
        self.release_dns.set()
        self.lookups[0].join(timeout=1)
        self.assertFalse(self.lookups[0].is_alive())
        connection = self.fake_socket(code=0)
        self.sockets.side_effect = None
        self.sockets.return_value = connection
        future = client.call_async('ping')
        self.assertEqual(msgpack.unpackb(bytes(connection.send.call_args.args[0])), [0, 3, 'ping', []])
        client._dispatch([1, 3, None, True])
        self.assertTrue(future.get())
        self.dns.assert_called_once()

    def test_resolver_threads_are_bounded_across_clients(self):
        slots = threading.BoundedSemaphore(2)
        self.patch(rpc, '_RESOLVER_SLOTS', new=slots)
        self.block_dns(expected=2)
        callers = [self.run_background(self.client(timeout=0.04).call_async, 'ping') for unused in range(2)]
        self.assertTrue(self.lookup_started.wait(1))
        for caller in callers:
            with self.assertRaises(RPCTimeoutError):
                caller.result(timeout=1).get()
        for unused in range(3):
            caller = self.run_background(self.client(timeout=0.02).call_async, 'ping')
            with self.assertRaises(RPCTimeoutError):
                caller.result(timeout=1).get()
        self.assertEqual(len(self.lookups), 2)
        self.assertTrue(all(worker.daemon for worker in self.lookups))
        self.assertFalse(slots.acquire(blocking=False))
        # Numeric addresses still work while all DNS slots are occupied.
        numeric = self.client('127.0.0.1')
        self.assertEqual(numeric._resolve(time.monotonic() + 1)[0][4], ('127.0.0.1', 41451))
        self.sockets.assert_not_called()

    def test_dns_and_all_addresses_share_one_absolute_deadline(self):
        now = [0.0]
        self.patch(rpc.time, 'monotonic', side_effect=lambda: now[0])

        def resolve(*args):
            now[0] += 0.25
            return self.addresses

        self.dns.side_effect = resolve
        first = self.fake_socket(error=errno.ECONNREFUSED)
        second = self.fake_socket()
        self.sockets.side_effect = [first, second]
        waits = []

        def select_sockets(readable, writable, exceptional, timeout):
            waits.append(timeout)
            now[0] += timeout
            # Windows may report a refused connect in the exceptional set.
            return ([], [], [first]) if writable == [first] else ([], [], [])

        self.select.side_effect = select_sockets
        client = self.client(timeout=1)
        future = client.call_async('ping')
        with self.assertRaises(RPCTimeoutError):
            future.get()
        self.assertAlmostEqual(now[0], 1.0)
        self.assertAlmostEqual(sum(waits), 0.75)
        self.assertTrue(all(0 < wait <= 0.1 for wait in waits))
        self.assertEqual(self.sockets.call_count, 2)  # No fresh budget for address three.
        first.connect_ex.assert_called_once_with(self.addresses[0][4])
        second.connect_ex.assert_called_once_with(self.addresses[1][4])
        self.assertTrue(first.closed.is_set())
        self.assertTrue(second.closed.is_set())
        self.assertIsNone(client._connecting_socket)
        self.assertEqual(client._pending, {})

    def test_failed_address_falls_back_without_replaying_request(self):
        self.dns.side_effect = None
        self.dns.return_value = self.addresses
        first = self.fake_socket(code=errno.ECONNREFUSED)
        second = self.fake_socket()
        self.sockets.side_effect = [first, second]
        client = self.client()
        future = client.call_async('echo', 8)
        self.assertTrue(first.closed.is_set())
        first.send.assert_not_called()
        second.send.assert_called_once()
        second.getsockopt.assert_called_once_with(socket.SOL_SOCKET, socket.SO_ERROR)
        self.assertIs(client._socket, second)
        self.assertIsNone(client._connecting_socket)
        client._dispatch([1, 0, None, 8])
        self.assertEqual(future.get(), 8)

    def test_dns_failure_releases_resolution_for_next_attempt(self):
        self.dns.side_effect = socket.gaierror('No such host')
        client = self.client()
        with self.assertRaises(RPCTransportError):
            client.call('ping')
        self.assertIsNone(client._resolution)
        self.dns.side_effect = None
        self.dns.return_value = self.addresses
        connection = self.fake_socket(code=0)
        self.sockets.side_effect = None
        self.sockets.return_value = connection
        future = client.call_async('ping')
        client._dispatch([1, 1, None, True])
        self.assertTrue(future.get())
        self.assertEqual(self.dns.call_count, 2)

    def test_queued_sender_wait_is_sliced_and_deadline_bounded(self):
        now = [0.0]
        self.patch(rpc.time, 'monotonic', side_effect=lambda: now[0])
        client = self.client(timeout=0.25)
        waits = []

        def acquire(timeout):
            waits.append(timeout)
            now[0] += timeout
            return False

        client._send_lock = mock.Mock(acquire=mock.Mock(side_effect=acquire))
        future = client.call_async('ping')
        with self.assertRaises(RPCTimeoutError):
            future.get()
        self.assertAlmostEqual(sum(waits), 0.25)
        self.assertTrue(all(0 < wait <= 0.1 for wait in waits))
        client._send_lock.release.assert_not_called()
        self.dns.assert_not_called()
        self.sockets.assert_not_called()

    def test_close_cancels_slow_dns_and_queued_sender(self):
        self.block_dns()
        client = self.client()
        client._send_lock = ObservedLock()
        first = self.run_background(client.call_async, 'ping')
        self.assertTrue(self.lookup_started.wait(1))
        second = self.run_background(client.call_async, 'ping')
        self.assertTrue(client._send_lock.contended.wait(1))
        client.close()
        self.assert_transport_failure(first)
        self.assert_transport_failure(second)
        self.assertFalse(self.release_dns.is_set())
        self.assertEqual(client._pending, {})
        self.sockets.assert_not_called()

    def test_close_owns_connecting_socket_and_wakes_queued_sender(self):
        client = self.client('127.0.0.1')
        client._send_lock = ObservedLock()
        connection = self.fake_socket()
        self.sockets.side_effect = None
        self.sockets.return_value = connection
        connecting = threading.Event()

        def select_sockets(readable, writable, exceptional, timeout):
            connecting.set()
            connection.closed.wait(timeout)
            return [], [], []

        self.select.side_effect = select_sockets
        first = self.run_background(client.call_async, 'ping')
        self.assertTrue(connecting.wait(1))
        self.assertIs(client._connecting_socket, connection)
        second = self.run_background(client.call_async, 'ping')
        self.assertTrue(client._send_lock.contended.wait(1))
        client.close()
        self.assertTrue(connection.closed.is_set())
        self.assert_transport_failure(first)
        self.assert_transport_failure(second)
        connection.send.assert_not_called()
        self.assertIsNone(client._connecting_socket)
        self.assertIsNone(client._socket)
        self.assertEqual(client._pending, {})
        client.close()

    def test_close_racing_connect_completion_does_not_install_socket(self):
        client = self.client('127.0.0.1')
        connection = self.fake_socket(code=0)
        connection.connect_ex.side_effect = lambda address: client.close() or 0
        self.sockets.side_effect = None
        self.sockets.return_value = connection
        future = client.call_async('ping')
        with self.assertRaises(RPCTransportError):
            future.get()
        self.assertTrue(connection.closed.is_set())
        connection.send.assert_not_called()
        self.assertIsNone(client._socket)
        self.assertIsNone(client._connecting_socket)
        self.assertIsNone(client._reader)

    def test_close_cancels_wait_for_resolver_slot(self):
        slots = ObservedLock(held=True)
        self.patch(rpc, '_RESOLVER_SLOTS', new=slots)
        client = self.client()
        caller = self.run_background(client.call_async, 'ping')
        self.assertTrue(slots.contended.wait(1))
        client.close()
        self.assert_transport_failure(caller)
        self.dns.assert_not_called()
        self.sockets.assert_not_called()

    def test_close_alone_completes_an_already_registered_future(self):
        self.block_dns()
        client = self.client()
        caller = self.run_background(client.call_async, 'ping')
        self.assertTrue(self.lookup_started.wait(1))
        pending = next(iter(client._pending.values()))
        complete = pending._future.set_exception
        close_owns_completion = threading.Event()
        allow_completion = threading.Event()

        def pause_completion(error):
            close_owns_completion.set()
            if not allow_completion.wait(2):
                raise AssertionError('Test did not release close')
            complete(error)

        self.patch(pending._future, 'set_exception', side_effect=pause_completion)
        closer = self.run_background(client.close)
        try:
            self.assertTrue(close_owns_completion.wait(1))
            self.assertIs(caller.result(timeout=1), pending)
            self.assertFalse(pending._future.done())
            pending._future.set_exception.assert_called_once()
        finally:
            allow_completion.set()
        closer.result(timeout=1)
        with self.assertRaises(RPCTransportError):
            pending.get()


class FutureCompatibilityTests(unittest.TestCase):
    def test_distinct_pre311_future_timeout_class_is_translated(self):
        class LegacyFutureTimeout(Exception):
            pass

        self.assertFalse(issubclass(LegacyFutureTimeout, TimeoutError))
        for method in ('get', 'join'):
            with self.subTest(method=method):
                future = Future(time.monotonic() + 1)
                wait_error = LegacyFutureTimeout('wait elapsed')
                handler = mock.Mock(return_value='ignored')
                future.attach_error_handler(handler)
                with mock.patch.object(rpc, '_FutureTimeoutError', LegacyFutureTimeout), \
                        mock.patch.object(future._future, 'result', side_effect=wait_error), \
                        mock.patch.object(future._future, 'done', side_effect=AssertionError('Racy done check')) as done:
                    with self.assertRaises(RPCTimeoutError) as caught:
                        getattr(future, method)(timeout=0)
                    self.assertIs(caught.exception.__cause__, wait_error)
                    done.assert_not_called()
                handler.assert_not_called()

    def test_completion_racing_wait_timeout_is_always_translated(self):
        for method in ('get', 'join'):
            for completion in ('result', 'error', 'rpc_timeout'):
                with self.subTest(method=method, completion=completion):
                    future = Future(time.monotonic() + 1)
                    handler = mock.Mock(return_value='ignored')
                    future.attach_error_handler(handler)
                    wait_error = concurrent.futures.TimeoutError('wait elapsed')

                    def finish_then_timeout(timeout):
                        if completion == 'result':
                            future._future.set_result(42)
                        else:
                            error = RPCError('remote error') if completion == 'error' else RPCTimeoutError('request expired')
                            future._future.set_exception(error)
                        raise wait_error

                    with mock.patch.object(future._future, 'result', side_effect=finish_then_timeout), \
                            mock.patch.object(future._future, 'done', side_effect=AssertionError('Racy done check')) as done:
                        with self.assertRaises(RPCTimeoutError) as caught:
                            getattr(future, method)(timeout=0)
                        self.assertIs(caught.exception.__cause__, wait_error)
                        done.assert_not_called()
                    self.assertTrue(future._future.done())
                    handler.assert_not_called()
                    if completion == 'result':
                        self.assertEqual(future.get(), 42)
                    else:
                        self.assertIsNone(future.get())
                        handler.assert_called_once_with(future.error)

    def test_rpc_errors_precede_wait_timeouts_and_handler_return_is_ignored(self):
        for error in (RPCError('remote'), RPCTransportError('closed'), RPCTimeoutError('expired')):
            with self.subTest(error=type(error).__name__):
                future = Future(time.monotonic() + 1)
                future._future.set_exception(error)
                with self.assertRaises(RPCError) as caught:
                    future.get()
                self.assertIs(caught.exception, error)
                self.assertIsNone(future.join())
                handler = mock.Mock(return_value=42)
                future.attach_error_handler(handler)
                self.assertIsNone(future.get())
                handler.assert_called_once_with(error)

    def test_pending_wait_timeout_does_not_handle_or_cancel_future(self):
        for method in ('get', 'join'):
            with self.subTest(method=method):
                future = Future(time.monotonic() + 1)
                handler = mock.Mock()
                future.attach_error_handler(handler)
                with self.assertRaises(RPCTimeoutError):
                    getattr(future, method)(timeout=0)
                self.assertFalse(future._future.done())
                handler.assert_not_called()
                future._future.set_result(42)
                self.assertEqual(future.get(), 42)
                self.assertIsNone(future.join())

    def test_result_handler_return_is_ignored_and_join_does_not_invoke_it(self):
        future = Future(time.monotonic() + 1)
        future._future.set_result(42)
        handler = mock.Mock(return_value='ignored')
        future.attach_result_handler(handler)
        self.assertIsNone(future.join())
        handler.assert_not_called()
        self.assertIsNone(future.get())
        handler.assert_called_once_with(42)
        self.assertEqual(future.result, 42)


if __name__ == '__main__':
    unittest.main()
