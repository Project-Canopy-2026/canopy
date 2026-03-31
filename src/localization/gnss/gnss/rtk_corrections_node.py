import socket
import time
import threading
import base64
import serial

import rclpy
from rclpy.node import Node

class ntrip_client(Node):
    def __init__(self):
        
<<<<<<< HEAD
        super().__init__('ntripclient')
=======
        super().__init__('ntrip_client')
>>>>>>> dev/localization/neha

        #self.rtcm_topic = '/rtcm'

        self.declare_parameter('server', 'rtk2go.com')
        self.declare_parameter('port', 2101)
        self.declare_parameter('user', 'nkayiti@andrew.cmu.edu')
        self.declare_parameter('password', '')
        self.declare_parameter('mountpoint', 'cmuairlab01')
        
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baud', 115200)

        self.user = self.get_parameter('user').get_parameter_value().string_value
        self.password = self.get_parameter('password').get_parameter_value().string_value
        self.server = self.get_parameter('server').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.mountpoint = self.get_parameter('mountpoint').get_parameter_value().string_value

        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.serial_baud = self.get_parameter('serial_baud').get_parameter_value().integer_value

        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)

        self.serial_conn = None

        self.thread.start()


    def ntrip_connect(self) -> socket.socket:
        s = socket.create_connection((self.server, self.port), timeout=10.0)
        s.settimeout(10.0)

        s.sendall(self.build_ntrip_request())

        header = b""
        # read response headers until \r\n\r\n 
        while b"\n" not in header:
            chunk = s.recv(1)
            if not chunk:
                raise RuntimeError("NTRIP connection closed before headers completed")
            header += chunk
            if len(header) > 8192:
                raise RuntimeError("NTRIP headers too large / invalid")

        header_text = header.decode('latin-1', errors='replace')
        if "200" not in header_text and "ICY 200" not in header_text:
            raise RuntimeError(f"NTRIP request rejected. Header was:\n{header_text}")

        self.get_logger().info("NTRIP connected, streaming RTCM")
        s.settimeout(2.0)
        return s


    def build_ntrip_request(self):
        # returns byte object that looks like HTTP request
        credentials = f"{self.user}:{self.password}"
        auth_b64 = base64.b64encode(credentials.encode('utf-8')).decode('ascii')

        req_lines = [
            f"GET /{self.mountpoint} HTTP/1.0",
            f"Host: {self.server}",
            f"User-Agent: NTRIP PythonClient/1.0",
            "Accept: */*",
            f"Authorization: Basic {auth_b64}",
            "Connection: close",
            "",
            "",
        ]
        return ("\r\n".join(req_lines)).encode('utf-8')

    
    def open_serial(self):
        if self.serial_conn and self.serial_conn.is_open:
            return

        self.serial_conn = serial.Serial(
            port = self.serial_port,
            baudrate = self.serial_baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0,
            write_timeout=1.0,
        )
        self.get_logger().info("serial open")


    def destroy_node(self):
        self.stop.set()
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
        except Exception:
            pass
        super().destroy_node()


    def run(self):
        # response = self.send_ntrip_message()

        # while not response
        #     time.sleep(10)
        #     self.send_ntrip_message()

        bytes_forwarded = 0

        while not self.stop.is_set():
            sock = None
            try:
                self.open_serial()
                sock = self.ntrip_connect()

                while not self.stop.is_set():
                    data = sock.recv(4096)
                    if not data:
                        raise RuntimeError("NTRIP stream ended")
                    
                    # Forward data to GNSS reciever
                    self.serial_conn.write(data)
                    bytes_forwarded += len(data)

                   # self.get_logger().info(f"Forwarded {bytes_forwarded} bytes RTCM -> serial")

            except Exception as e:
                self.get_logger().warn(f"streaming issue: {e}. reconnecting in 2 seconds")
                
                # try to close socket
                try:
                    if sock:
                        sock.close()
                except Exception:
                    pass

                time.sleep(2.0)

        # try to close socket
                try:
                    if sock:
                        sock.close()
                except Exception:
                    pass


def main(args=None):
    rclpy.init(args=args)

    node = ntrip_client()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
