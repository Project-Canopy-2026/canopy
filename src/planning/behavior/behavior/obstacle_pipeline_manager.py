import ctypes
import os
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# ROS2 message definitions
from std_msgs.msg import Bool

# Lidar obstacle pipeline nodes owned by this manager: (node name, package, executable, arguments).
# Their parameters come from params_file, keyed by node name.
# occupancy_grid_node is not here; it stays running and pauses itself on /behavior/is_planting.
MANAGED_NODES = [
    ("velodyne_driver", "velodyne_driver", "velodyne_driver_node", []),
    ("velodyne_convert", "velodyne_pointcloud", "velodyne_transform_node", []),
    ("ground_segmentation", "patchworkpp", "demo", ["/velodyne_points"]),
]

PR_SET_PDEATHSIG = 1


def dieWithParent():
    # Runs in the child before exec. If this manager dies, even from SIGKILL, the
    # kernel sends the child SIGTERM, so no lidar node is left running on its own.
    # Children must be started from the main thread (rclpy.spin is single threaded),
    # because the signal fires when the thread that started them exits.
    ctypes.CDLL("libc.so.6", use_errno=True).prctl(PR_SET_PDEATHSIG, signal.SIGTERM)


class ObstaclePipelineManager(Node):
    """Stops the lidar obstacle pipeline while planting and restarts it afterwards.

    During planting the planner holds still, so obstacle data is not needed, and the
    lidar processing load slows the arm's MoveIt messages on the Jetson.
    """

    def __init__(self):
        super().__init__("obstacle_pipeline_manager")

        self.setUpParameters()

        self.commands = {}
        for name, package, executable, arguments in MANAGED_NODES:
            path = os.path.join(get_package_prefix(package), "lib", package, executable)
            if not os.access(path, os.X_OK):
                raise RuntimeError(f"Executable for {name} not found: {path}")
            self.commands[name] = [path, *arguments, "--ros-args",
                                   "-r", f"__node:={name}", "--params-file", self.params_file]

        self.processes = {}  # node name -> subprocess.Popen
        self.reported_exits = set()
        self.is_planting = False

        self.create_subscription(Bool, "/behavior/is_planting", self.isPlantingCb, 1)
        self.create_timer(1.0, self.checkProcesses)

        self.startPipeline()

    def isPlantingCb(self, msg: Bool):
        # The FSM republishes this at 10 Hz; only act on changes.
        if msg.data == self.is_planting:
            return
        self.is_planting = msg.data

        if self.is_planting:
            self.get_logger().info("Planting started: stopping the lidar obstacle pipeline")
            self.stopPipeline()
        else:
            self.get_logger().info("Planting finished: starting the lidar obstacle pipeline")
            self.startPipeline()

    def startPipeline(self):
        self.reported_exits.clear()
        for name, command in self.commands.items():
            process = self.processes.get(name)
            if process is not None and process.poll() is None:
                continue
            self.processes[name] = subprocess.Popen(command, preexec_fn=dieWithParent)
            self.get_logger().info(f"Started {name} (pid {self.processes[name].pid})")

    def stopPipeline(self):
        running = {name: p for name, p in self.processes.items() if p.poll() is None}
        for process in running.values():
            process.send_signal(signal.SIGINT)

        deadline = time.monotonic() + self.stop_timeout_sec
        for name, process in running.items():
            try:
                process.wait(timeout=max(0.0, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                self.get_logger().warn(f"{name} did not exit on SIGINT; killing it")
                process.kill()
                process.wait()
            self.get_logger().info(f"Stopped {name} (exit code {process.returncode})")

        self.processes.clear()

    def checkProcesses(self):
        # A node that dies while it should be running is reported, not restarted
        # (same as launch). It is started again after the next planting.
        if self.is_planting:
            return
        for name, process in self.processes.items():
            if process.poll() is not None and name not in self.reported_exits:
                self.reported_exits.add(name)
                self.get_logger().error(
                    f"{name} exited unexpectedly (exit code {process.returncode}); "
                    "obstacle data may be missing until the next planting"
                )

    def setUpParameters(self):
        self.params_file = self.declare_parameter("params_file", "").value
        self.stop_timeout_sec = float(self.declare_parameter("stop_timeout_sec", 5.0).value)

        if not os.path.isfile(self.params_file):
            raise RuntimeError(f"params_file not found: '{self.params_file}'")


def main(args=None):
    rclpy.init(args=args)

    node = ObstaclePipelineManager()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stopPipeline()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
