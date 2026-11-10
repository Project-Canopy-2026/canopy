"""Tests for w200_watchdog: when it restarts domain_bridge, and what it kills."""
import os
import shutil
import signal
import subprocess
import tempfile
import time
import unittest
import uuid
from pathlib import Path
from unittest.mock import MagicMock, patch

import rclpy

from gnss import w200_watchdog
from gnss.w200_watchdog import W200Watchdog


class TestWatchdogTiming(unittest.TestCase):
    """Restart decisions, with a fake monotonic clock and killBridge mocked out."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.t = 0.0
        self.clock = patch.object(w200_watchdog.time, 'monotonic', side_effect=lambda: self.t)
        self.clock.start()
        # No real subscription or timer: a live W200 IMU must not reach the test.
        with patch.object(W200Watchdog, 'create_subscription'), \
                patch.object(W200Watchdog, 'create_timer'):
            self.node = W200Watchdog()
        self.node.killBridge = MagicMock()
        self.kill_times = []
        self.node.killBridge.side_effect = lambda: self.kill_times.append(self.t)

    def tearDown(self):
        self.node.destroy_node()
        self.clock.stop()

    def run_until(self, end, data_until=None, data_from=None):
        """Advance to `end` in 0.1 s steps; IMU at 10 Hz in [data_from, data_until)."""
        step = 0
        while self.t < end - 1e-9:
            self.t = round(self.t + 0.1, 6)
            step += 1
            gets_data = (data_from is None or self.t >= data_from) and \
                (data_until is None or self.t < data_until)
            if (data_from is not None or data_until is not None) and gets_data:
                self.node.dataCb(None)
            if step % 5 == 0:  # the real timer runs at 2 Hz
                self.node.check()

    def test_no_restart_during_startup_grace(self):
        self.run_until(21.5)
        self.assertEqual(self.kill_times, [])

    def test_restarts_after_grace_plus_timeout_without_data(self):
        self.run_until(23.0)
        self.assertEqual(len(self.kill_times), 1)
        self.assertAlmostEqual(self.kill_times[0], 22.0, delta=0.5)

    def test_no_restart_while_data_flows(self):
        self.run_until(300.0, data_from=0.0)
        self.assertEqual(self.kill_times, [])

    def test_restarts_two_seconds_after_data_stops(self):
        # Like 11-9: data flowing, then the bridge stops forwarding.
        self.run_until(60.0, data_from=0.0, data_until=40.0)
        self.assertEqual(len(self.kill_times), 1)
        self.assertAlmostEqual(self.kill_times[0], 42.0, delta=0.5)

    def test_backoff_doubles_up_to_cap_when_w200_is_off(self):
        self.run_until(600.0)
        gaps = [b - a for a, b in zip(self.kill_times, self.kill_times[1:])]
        # wait after each restart: 20 -> 40 -> 80 -> 120 (cap), each plus the 2 s timeout
        expected = [22.0, 42.0, 82.0, 122.0, 122.0]
        self.assertGreaterEqual(len(gaps), len(expected))
        for gap, exp in zip(gaps, expected):
            self.assertAlmostEqual(gap, exp, delta=0.5)
        self.assertTrue(all(g <= 122.5 for g in gaps))

    def test_backoff_resets_when_data_returns(self):
        self.run_until(150.0)  # W200 off: several restarts, grace has grown
        self.assertGreater(self.node.current_grace, self.node.grace)
        self.run_until(160.0, data_from=150.0)
        self.assertEqual(self.node.current_grace, self.node.grace)


class TestWatchdogKill(unittest.TestCase):
    """
    Test findBridgePids/killBridge against real stand-in processes.

    Uses a unique config name so a real domain_bridge on this machine is never
    matched or killed.
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.config = f'test_{uuid.uuid4().hex[:8]}_w200_bridge.yaml'
        self.procs = []
        with patch.object(W200Watchdog, 'create_subscription'), \
                patch.object(W200Watchdog, 'create_timer'):
            self.node = W200Watchdog()
        self.node.bridge_config = self.config

    def tearDown(self):
        self.node.destroy_node()
        for p in self.procs:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            p.wait()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def start(self, exe, *args):
        p = subprocess.Popen([str(exe), '-c', 'sleep 30; true', *args],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                             start_new_session=True)
        self.procs.append(p)
        return p

    def start_fake_bridge(self):
        """Process named domain_bridge with the config path in its arguments."""
        exe = self.tmp / 'domain_bridge'
        if not exe.exists():
            shutil.copy('/bin/bash', exe)
        return self.start(exe, 'x', str(self.tmp / self.config))

    def wait_until_ready(self, proc):
        for _ in range(50):
            try:
                if (Path(f'/proc/{proc.pid}/comm').read_text().strip()
                        in ('domain_bridge', 'bash')):
                    return
            except OSError:
                pass
            time.sleep(0.02)

    def test_finds_bridge_by_process_name_and_config(self):
        bridge = self.start_fake_bridge()
        self.wait_until_ready(bridge)
        self.assertIn(bridge.pid, self.node.findBridgePids())

    def test_ignores_shell_that_only_mentions_the_bridge(self):
        # e.g. a terminal running: grep "domain_bridge .*w200_bridge.yaml"
        decoy = self.start('/bin/bash', 'domain_bridge', str(self.tmp / self.config))
        self.wait_until_ready(decoy)
        self.assertNotIn(decoy.pid, self.node.findBridgePids())

    def test_ignores_bridge_with_other_config(self):
        self.node.bridge_config = 'some_other_bridge.yaml'
        bridge = self.start_fake_bridge()
        self.wait_until_ready(bridge)
        self.assertNotIn(bridge.pid, self.node.findBridgePids())

    def test_never_matches_itself(self):
        self.assertNotIn(os.getpid(), self.node.findBridgePids())

    def test_kill_bridge_kills_it_and_saves_thread_states(self):
        bridge = self.start_fake_bridge()
        decoy = self.start('/bin/bash', 'domain_bridge', str(self.tmp / self.config))
        self.wait_until_ready(bridge)
        self.wait_until_ready(decoy)
        (self.tmp / '.ros' / 'log').mkdir(parents=True)

        with patch.object(w200_watchdog.Path, 'home', return_value=self.tmp):
            self.node.killBridge()

        self.assertEqual(bridge.wait(timeout=5), -signal.SIGKILL)
        self.assertIsNone(decoy.poll(), 'decoy shell must survive')
        dumps = list((self.tmp / '.ros' / 'log').glob(f'w200_bridge_stuck_*_{bridge.pid}.txt'))
        self.assertEqual(len(dumps), 1)
        self.assertIn('domain_bridge', dumps[0].read_text())

    def test_kill_bridge_with_no_bridge_running_is_harmless(self):
        self.node.killBridge()  # must not raise


if __name__ == '__main__':
    unittest.main()
