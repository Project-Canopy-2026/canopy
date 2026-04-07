#!/usr/bin/env python3
# ============================================================
#  Serial Bridge Node — planting_controller
#  Translates between ROS2 topics and the Arduino UNO R3
#  over USB serial (/dev/ttyUSB0 @ 115200 baud).
#
#  Subscribes:   /arduino_cmd    (std_msgs/String)
#                  → writes raw command string to serial + '\n'
#
#  Publishes:    /arduino_status (std_msgs/String)
#                  → every line received from Arduino
#
#  Arduino reply tokens this node passes through unchanged:
#    "ACK:<cmd>"     – command received and dispatched
#    "DONE:STEPPER"  – timed stepper move completed
#    "ERR:<reason>"  – malformed / unknown command
#    "READY"         – Arduino finished setup(), safe to send
#
#  Launch parameter:
#    port     (string)  default '/dev/ttyUSB0'
#    baudrate (int)     default 115200
#
#  Common setup issues:
#    Permission denied → sudo usermod -aG dialout $USER  (re-login)
#    Wrong port        → ls /dev/ttyUSB*  or  dmesg | tail -20
# ============================================================

import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import serial.serialutil

class SerialBridgeNode(Node):
    
    def __init__(self):
        super().__init__('serial_bridge')

        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # ── Publisher: Arduino → ROS2 ─────────────────────────────────
        self.status_pub = self.create_publisher(String, '/arduino_status', 10)

        # ── Subscriber: ROS2 → Arduino ────────────────────────────────
        self.cmd_sub = self.create_subscription(
            String,
            '/arduino_cmd',
            self._cmd_callback,
            10
        )

        # ── Open serial port ─────────────────────────────────────────
        # The UNO R3 resets on DTR assertion (which pyserial triggers
        # when opening the port).  We wait for the Arduino to send
        # "READY" before the bridge announces itself as ready, so the
        # FSM node knows it is safe to start sending commands.
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=1.0          # readline() blocks at most 1 s
            )
            self.get_logger().info(
                f'Serial port {port} opened at {baudrate} baud. '
                f'Waiting for Arduino READY...'
            )
        except serial.serialutil.SerialException as exc:
            self.get_logger().fatal(f'Cannot open {port}: {exc}')
            raise SystemExit(1)

        # ── Serial write lock (cmd_callback runs on ROS spin thread) ──
        # read_loop runs on its own thread; writes must be serialised.
        self._write_lock = threading.Lock()

        # ── Background reader thread ──────────────────────────────────
        self._arduino_ready = False
        self._read_thread = threading.Thread(
            target=self._read_loop,
            daemon=True,
            name='serial_read'
        )
        self._read_thread.start()

    def destroy_node(self):
        """Clean shutdown — close serial port before ROS2 tears down."""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Serial port closed.')
        super().destroy_node()

    # ------------------------------------------------------------------
    #  ROS2 → Arduino  (subscriber callback, runs on spin thread)
    # ------------------------------------------------------------------

    def _cmd_callback(self, msg: String):
        """
        Forward an /arduino_cmd message to the Arduino over serial.
        Thread-safe via _write_lock.
        """
        if not self._arduino_ready:
            self.get_logger().warn(
                f'Arduino not ready yet — dropping command: {msg.data}'
            )
            return

        command = msg.data.strip() + '\n'
        try:
            with self._write_lock:
                self.ser.write(command.encode('utf-8'))
            self.get_logger().debug(f'→ Arduino: {msg.data}')
        except serial.serialutil.SerialException as exc:
            self.get_logger().error(f'Serial write error: {exc}')

    # ------------------------------------------------------------------
    #  Arduino → ROS2  (background thread)
    # ------------------------------------------------------------------

    def _read_loop(self):
        """
        Continuously reads newline-terminated strings from the Arduino
        and publishes them on /arduino_status.

        Runs in a daemon thread so it exits automatically when the
        process terminates.
        """
        while rclpy.ok():
            try:
                raw = self.ser.readline()          # blocks up to timeout= seconds
                if not raw:
                    continue                        # timeout with no data — loop

                line = raw.decode('utf-8', errors='replace').strip()
                if not line:
                    continue

                self.get_logger().debug(f'← Arduino: {line}')

                # ── Special case: Arduino just booted ────────────────
                if line == 'READY':
                    self._arduino_ready = True
                    self.get_logger().info('Arduino is READY — bridge active.')

                # ── Publish every line, including READY ───────────────
                # The FSM node can subscribe to /arduino_status and
                # react to "READY" itself if it needs to gate on startup.
                msg = String()
                msg.data = line
                self.status_pub.publish(msg)

            except serial.serialutil.SerialException as exc:
                self.get_logger().error(f'Serial read error: {exc}')
                break                              # fatal — exit thread
            except UnicodeDecodeError:
                self.get_logger().warn('Received non-UTF-8 bytes from Arduino — skipped.')


# ============================================================
#  Entry point
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()