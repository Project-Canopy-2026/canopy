#!/usr/bin/env python3
"""Start and stop the Canopy robot stack from Foxglove.

Runs `launch/canopy_launch.launch.py` as a background process when asked, so the
"Start planting" / "Stop" buttons in the Foxglove Seedling Planner panel can
control it.

Interface (default node name canopy_launcher):
    /canopy_launcher/start   std_srvs/Trigger   launch the stack (no-op if running)
    /canopy_launcher/stop    std_srvs/Trigger   SIGINT the stack, escalating to
                                                SIGTERM / SIGKILL if it hangs
    /canopy_launcher/status  std_msgs/String    JSON, 1 Hz:
        {"state": "stopped|running|stopping", "pid": int|null, "uptime_s": float,
         "last_exit_code": int|null, "log_file": str, "command": str}

The stack is launched with foxglove:=false, because foxglove_bridge is already
running next to this node (see launch/foxglove_operator.launch.py) and a second
bridge would fight over port 8765.

Output of the stack goes to ~/.ros/canopy_launcher/canopy_<timestamp>.log, and is
also printed to this node's terminal (turn off with -p echo_output:=false).
Stopping this node also stops the stack.

Run:
    ros2 launch <repo>/launch/foxglove_operator.launch.py
"""
import datetime
import json
import os
import re
import shlex
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
DEFAULT_COMMAND = (
    f"ros2 launch {os.path.join(REPO_ROOT, 'launch', 'canopy_launch.launch.py')} foxglove:=false"
)
# Terminal color codes; kept on screen, stripped from the log file so it greps cleanly.
ANSI_ESCAPE = re.compile(rb"\x1b\[[0-?]*[ -/]*[@-~]")


class CanopyLauncher(Node):
    def __init__(self):
        super().__init__("canopy_launcher")

        self.declare_parameter("launch_command", DEFAULT_COMMAND)
        self.declare_parameter("log_dir", os.path.expanduser("~/.ros/canopy_launcher"))
        self.declare_parameter("sigint_timeout_s", 15.0)
        self.declare_parameter("echo_output", True)

        self.lock = threading.Lock()
        self.proc = None
        self.start_time = None
        self.stopping = False
        self.last_exit_code = None
        self.log_file = ""

        self.create_service(Trigger, "~/start", self.startCb)
        self.create_service(Trigger, "~/stop", self.stopCb)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, "~/status", latched)
        self.create_timer(1.0, self.publishStatus)

        self.get_logger().info(f"Ready. Start will run: {self.command()}")

    def command(self):
        return self.get_parameter("launch_command").value

    # ------------------------------------------------------------ services

    def startCb(self, request, response):
        with self.lock:
            self.reapLocked()
            if self.proc is not None:
                response.success = False
                response.message = f"Already running (pid {self.proc.pid})"
                return response

            log_dir = self.get_parameter("log_dir").value
            os.makedirs(log_dir, exist_ok=True)
            stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            self.log_file = os.path.join(log_dir, f"canopy_{stamp}.log")
            log_handle = None
            try:
                log_handle = open(self.log_file, "wb")
                env = dict(os.environ)
                # Output goes through a pipe, not a terminal: without these,
                # Python nodes print in delayed chunks and ROS drops the colors.
                env["PYTHONUNBUFFERED"] = "1"
                env["RCUTILS_COLORIZED_OUTPUT"] = "1"
                # New session so we can signal the whole process group, and so
                # Ctrl-C in this terminal reaches the stack only through stop().
                self.proc = subprocess.Popen(
                    shlex.split(self.command()),
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL,
                    start_new_session=True,
                    env=env,
                )
            except Exception as e:  # noqa: BLE001 - report any failure to the UI
                if log_handle is not None:
                    log_handle.close()
                self.proc = None
                response.success = False
                response.message = f"Failed to start: {e}"
                self.get_logger().error(response.message)
                return response

            threading.Thread(
                target=self.copyOutput,
                args=(self.proc.stdout, log_handle, bool(self.get_parameter("echo_output").value)),
                daemon=True,
            ).start()
            self.start_time = time.monotonic()
            self.stopping = False
            self.last_exit_code = None
            response.success = True
            response.message = f"Started (pid {self.proc.pid}), log: {self.log_file}"
            self.get_logger().info(response.message)
        self.publishStatus()
        return response

    def stopCb(self, request, response):
        with self.lock:
            self.reapLocked()
            if self.proc is None:
                response.success = True
                response.message = "Not running"
                return response
            if self.stopping:
                response.success = True
                response.message = "Already stopping"
                return response
            self.stopping = True
            proc = self.proc

        threading.Thread(target=self.stopProcess, args=(proc,), daemon=True).start()
        response.success = True
        response.message = f"Stopping (pid {proc.pid})"
        self.get_logger().info(response.message)
        self.publishStatus()
        return response

    # ------------------------------------------------------------ process

    @staticmethod
    def copyOutput(pipe, log_handle, echo):
        """Copy stack output to the log file and, if echo, to this terminal.

        Runs until the stack (and every process holding its output) exits.
        """
        try:
            for line in iter(pipe.readline, b""):
                log_handle.write(ANSI_ESCAPE.sub(b"", line))
                log_handle.flush()
                if echo:
                    sys.stdout.buffer.write(line)
                    sys.stdout.buffer.flush()
        except (OSError, ValueError):
            pass  # terminal or file went away during shutdown
        finally:
            pipe.close()
            log_handle.close()

    def stopProcess(self, proc):
        """SIGINT the process group, like Ctrl-C, then escalate if it hangs."""
        timeout = float(self.get_parameter("sigint_timeout_s").value)
        for sig, wait_s in ((signal.SIGINT, timeout), (signal.SIGTERM, 5.0), (signal.SIGKILL, 5.0)):
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                break
            try:
                proc.wait(timeout=wait_s)
                break
            except subprocess.TimeoutExpired:
                self.get_logger().warn(f"Stack did not exit after {sig.name}, escalating")
        with self.lock:
            self.reapLocked()

    def reapLocked(self):
        """Clear state if the child has exited. Caller holds self.lock."""
        if self.proc is None:
            return
        code = self.proc.poll()
        if code is None:
            return
        self.last_exit_code = code
        self.get_logger().info(f"Stack exited with code {code}")
        self.proc = None
        self.start_time = None
        self.stopping = False

    def publishStatus(self):
        with self.lock:
            self.reapLocked()
            if self.proc is None:
                state = "stopped"
            elif self.stopping:
                state = "stopping"
            else:
                state = "running"
            status = {
                "state": state,
                "pid": self.proc.pid if self.proc else None,
                "uptime_s": time.monotonic() - self.start_time if self.start_time else 0.0,
                "last_exit_code": self.last_exit_code,
                "log_file": self.log_file,
                "command": self.command(),
            }
        self.status_pub.publish(String(data=json.dumps(status)))

    def shutdownStack(self):
        with self.lock:
            proc = self.proc
        if proc is not None and proc.poll() is None:
            self.get_logger().info("Shutting down: stopping stack")
            self.stopProcess(proc)


def raiseInterrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    rclpy.init()
    node = CanopyLauncher()
    # Treat SIGTERM (sent by ros2 launch on shutdown) like Ctrl-C so the stack
    # is stopped instead of orphaned.
    signal.signal(signal.SIGTERM, raiseInterrupt)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Don't let a second Ctrl-C / SIGTERM cut the cleanup short.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.shutdownStack()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
