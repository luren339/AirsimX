from pathlib import Path
import socketserver
import struct
import sys
import threading
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import airsim
import msgpack


class LegacyImages(socketserver.BaseRequestHandler):
    def handle(self):
        unpacker = msgpack.Unpacker(raw=False)
        while True:
            block = self.request.recv(4096)
            if not block:
                return
            unpacker.feed(block)
            for kind, request_id, method, arguments in unpacker:
                rows = [{'image_data_uint8': data, 'image_data_float': [], 'width': 2, 'height': 1,
                         'image_type': 0, 'pixels_as_float': False, 'compress': False, 'time_stamp': 123}
                        for data in (b'\0' * 6, b'\xff\x80\0ABC')]
                self.request.sendall(msgpack.packb([1, request_id, None, rows], use_bin_type=False))


class BinaryCompatibilityTests(unittest.TestCase):
    def test_text_and_surrogateescape_byte_helpers(self):
        self.assertEqual(airsim.string_to_uint8_array('ABCD').tolist(), [65, 66, 67, 68])
        self.assertEqual(airsim.string_to_uint8_array('é').tolist(), [195, 169])
        payload = b'\xff\x80\0ABC'
        text = payload.decode('utf-8', errors='surrogateescape')
        value = airsim.string_to_uint8_array(text)
        self.assertEqual(value.tobytes(), payload)
        value[0] = 1
        self.assertEqual(value[0], 1)

    def test_float_string_matches_original_binary_interpretation(self):
        expected = struct.unpack('f', b'ABCD')[0]
        self.assertEqual(float(airsim.string_to_float_array('ABCD')[0]), expected)
        data = struct.pack('f', 1.25)
        self.assertEqual(float(airsim.string_to_float_array(data.decode('utf-8', errors='surrogateescape'))[0]), 1.25)

    def test_legacy_multi_image_strings_become_bytes(self):
        server = socketserver.ThreadingTCPServer(('127.0.0.1', 0), LegacyImages)
        server.daemon_threads = True
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            with airsim.MultirotorClient('127.0.0.1', server.server_address[1], timeout_value=2) as client:
                images = client.simGetImages([airsim.ImageRequest('0', airsim.ImageType.Scene, False, False)])
                self.assertEqual(images[0].image_data_uint8, b'\0' * 6)
                self.assertEqual(images[1].image_data_uint8, b'\xff\x80\0ABC')
                for image in images:
                    self.assertIsInstance(image.image_data_uint8, bytes)
                    self.assertEqual(airsim.string_to_uint8_array(image.image_data_uint8).size, 6)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)


if __name__ == '__main__':
    unittest.main()
