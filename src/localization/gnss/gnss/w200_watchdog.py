"""
Restarts domain_bridge when W200 data stops arriving.

domain_bridge is the only link to the W200. On 11-9 it stopped forwarding in
both directions without exiting (and ignored SIGINT/SIGTERM). This node watches
a W200 topic and SIGKILLs the bridge when it goes quiet; launch respawns it.

Staleness uses Jetson receive time, not header stamps: the W200 clock is
seconds off from the Jetson's.
"""
import os
import signal
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class W200Watchdog(Node):
    def __init__(self):
        super().__init__("w200_watchdog")
        self.topic = self.declare_parameter("topic", "/w200_0120/sensors/imu_0/data").value
        # s of silence before restarting
        self.timeout = self.declare_parameter("timeout", 2.0).value
        # s for DDS discovery after a (re)start
        self.grace = self.declare_parameter("startup_grace", 20.0).value
        # backoff cap if the W200 is off
        self.max_grace = self.declare_parameter("max_grace", 120.0).value
        # The bridge is the process named bridge_name whose arguments include bridge_config.
        self.bridge_name = self.declare_parameter("bridge_name", "domain_bridge").value
        self.bridge_config = self.declare_parameter("bridge_config", "w200_bridge.yaml").value

        self.current_grace = self.grace
        # Don't check until discovery has had time.
        self.last_rx = time.monotonic() + self.current_grace
        self.restarts = 0

        # Best effort matches whatever QoS the bridge copies from the W200.
        self.create_subscription(Imu, self.topic, self.dataCb, qos_profile_sensor_data)
        self.create_timer(0.5, self.check)

    def dataCb(self, _msg):
        if self.restarts and self.current_grace != self.grace:
            self.get_logger().info("W200 data is back")
        self.last_rx = time.monotonic()
        self.current_grace = self.grace

    def check(self):
        silence = time.monotonic() - self.last_rx
        if silence < self.timeout:
            return
        self.restarts += 1
        self.get_logger().error(
            f"No {self.topic} for {silence:.1f} s. Restarting domain_bridge (#{self.restarts})")
        self.killBridge()
        # Wait for the respawned bridge to rediscover the W200; back off if the W200 is off.
        self.last_rx = time.monotonic() + self.current_grace
        self.current_grace = min(2 * self.current_grace, self.max_grace)

    def findBridgePids(self):
        # Match the process name exactly, not "pgrep -f": a shell or grep whose
        # command line mentions the bridge would match that too and get killed.
        pids = []
        for proc in Path("/proc").iterdir():
            if not proc.name.isdigit():
                continue
            try:
                if (proc / "comm").read_text().strip() != self.bridge_name:
                    continue
                args = (proc / "cmdline").read_bytes().split(b"\0")
            except OSError:
                continue  # exited or not ours
            if any(a.decode(errors="replace").endswith(self.bridge_config) for a in args):
                pids.append(int(proc.name))
        return pids

    def killBridge(self):
        pids = self.findBridgePids()
        if not pids:
            self.get_logger().warn("domain_bridge is not running; launch should respawn it")
            return
        for pid in pids:
            self.dumpThreads(pid)
            try:
                os.kill(pid, signal.SIGKILL)  # a stuck bridge ignores SIGINT/SIGTERM
            except ProcessLookupError:
                pass

    def dumpThreads(self, pid):
        """Record where each bridge thread is blocked, to find out why it hangs."""
        name = f"w200_bridge_stuck_{time.strftime('%Y%m%d-%H%M%S')}_{pid}.txt"
        path = Path.home() / ".ros" / "log" / name
        lines = []
        for task in sorted(Path(f"/proc/{pid}/task").glob("*")):
            try:
                comm = (task / "comm").read_text().strip()
                wchan = (task / "wchan").read_text().strip()
                state = (task / "stat").read_text().rsplit(") ", 1)[1].split()[0]
                lines.append(f"{task.name:>8} {comm:<16} state={state} wchan={wchan}")
            except OSError:
                pass
        try:
            path.write_text("\n".join(lines) + "\n")
            self.get_logger().info(f"Bridge thread states saved to {path}")
        except OSError as e:
            self.get_logger().warn(f"Could not save bridge thread states: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = W200Watchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
