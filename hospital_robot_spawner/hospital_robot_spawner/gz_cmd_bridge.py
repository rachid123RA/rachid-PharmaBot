#!/usr/bin/env python3
"""
gz_cmd_bridge.py — Bridge ROS2 /demo/cmd_vel → gz-harmonic
Lit les commandes de vitesse publiées par nav_bridge (ROS2)
et les envoie à gz sim via la CLI gz topic (10 Hz).
"""
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class GzCmdBridge(Node):

    def __init__(self):
        super().__init__('gz_cmd_bridge')
        self._vx = 0.0
        self._vz = 0.0
        self._lock = threading.Lock()
        self._active = True

        self.create_subscription(Twist, '/demo/cmd_vel', self._cb, 10)

        # Envoi continu à 10 Hz dans un thread séparé
        self._thread = threading.Thread(target=self._send_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            "gz_cmd_bridge démarré — /demo/cmd_vel (ROS2) → gz transport"
        )

    def _cb(self, msg: Twist):
        with self._lock:
            self._vx = msg.linear.x
            self._vz = msg.angular.z

    def _send_loop(self):
        while self._active:
            with self._lock:
                vx, vz = self._vx, self._vz

            proto = (
                f'linear: {{x: {vx:.4f} y: 0.0 z: 0.0}} '
                f'angular: {{x: 0.0 y: 0.0 z: {vz:.4f}}}'
            )
            subprocess.Popen(
                ['gz', 'topic', '-t', '/demo/cmd_vel',
                 '-m', 'gz.msgs.Twist', '-p', proto],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            time.sleep(0.1)   # 10 Hz


def main():
    rclpy.init()
    node = GzCmdBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._active = False
        rclpy.shutdown()


if __name__ == '__main__':
    main()
