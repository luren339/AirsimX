"""Bounded loopback TCP/UDP echo probe for the dedicated mirrored WSL SITL setup."""

import argparse
import json
import socket
import subprocess
import threading


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--distro', default='AirsimX-PX4-Ubuntu24')
    args = parser.parse_args()
    payload = b'airsimx-sitl-transport-check'
    errors = []
    with socket.socket() as tcp, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
        for server, port in ((tcp, 4560), (udp, 14540)):
            if hasattr(socket, 'SO_EXCLUSIVEADDRUSE'):
                server.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            server.settimeout(15)
            server.bind(('127.0.0.1', port))
        tcp.listen(1)

        def tcp_echo():
            try:
                client, address = tcp.accept()
                with client:
                    client.settimeout(10)
                    received = client.recv(128)
                    if received != payload:
                        raise RuntimeError('Unexpected probe TCP payload')
                    client.sendall(received)
            except Exception as error:
                errors.append(repr(error))

        def udp_echo():
            try:
                received, address = udp.recvfrom(128)
                if received != payload:
                    raise RuntimeError('Unexpected probe UDP payload')
                udp.sendto(received, address)
            except Exception as error:
                errors.append(repr(error))

        threads = [threading.Thread(target=target, daemon=True) for target in (tcp_echo, udp_echo)]
        for thread in threads:
            thread.start()
        code = '''import socket,json
payload=b"airsimx-sitl-transport-check"
with socket.create_connection(("127.0.0.1",4560),timeout=10) as tcp:
    tcp.sendall(payload)
    assert tcp.recv(128)==payload
with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as udp:
    udp.settimeout(10)
    udp.bind(("127.0.0.1",14580))
    udp.sendto(payload,("127.0.0.1",14540))
    assert udp.recvfrom(128)[0]==payload
print(json.dumps({"tcp_4560":True,"udp_14580_to_14540_and_reply":True}))
'''
        result = subprocess.run(['wsl.exe', '-d', args.distro, '-u', 'airsimx', '--exec', 'python3', '-I', '-c', code],
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=30)
        for thread in threads:
            thread.join(timeout=16)
        print(result.stdout.decode('utf-8', errors='replace'))
        if result.returncode or errors:
            raise RuntimeError(json.dumps({'guest_exit': result.returncode,
                'guest_error': result.stderr.decode('utf-8', errors='replace'), 'host_errors': errors}))


if __name__ == '__main__':
    main()
