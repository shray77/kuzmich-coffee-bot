#!/usr/bin/env python3
"""
cup_fetch_orchestrator.py — склеивает существующий approach-контур
(cup_center_then_adaptive_node, /cup_approach_x/start) с новым захватом
(g1_manip_node, /g1_manip/grasp_cup). Ничего не меняет в их логике —
только вызывает их сервисы и слушает /perception/cup_pose_map, как
любой сторонний клиент.

Требует запущенными (см. README пакета, шаги 1-6):
  realsense, tf, yolo26_cup_pose_node, udp exporter, sdk receiver,
  cup_center_then_adaptive_node, и теперь ещё g1_manip_node.

Запуск:
  python3 ~/cup_fetch_orchestrator.py --ros-args -p stop_x_m:=1.25 -p timeout_s:=40.0
"""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger


class CupFetchOrchestrator(Node):
    def __init__(self):
        super().__init__('cup_fetch_orchestrator')
        d = {'stop_x_m': 1.25, 'timeout_s': 40.0, 'pose_timeout_s': 1.5}
        for k, v in d.items():
            self.declare_parameter(k, v)
        for k in d:
            setattr(self, k, float(self.get_parameter(k).value))

        self.last_pose = None
        self.last_pose_t = 0.0
        self.create_subscription(PoseStamped, '/perception/cup_pose_map', self._pose_cb, 10)

        self.cli_start = self.create_client(Trigger, '/cup_approach_x/start')
        self.cli_stop = self.create_client(Trigger, '/cup_approach_x/stop')
        self.cli_grasp = self.create_client(Trigger, '/g1_manip/grasp_cup')

    def _pose_cb(self, msg):
        self.last_pose = msg
        self.last_pose_t = time.time()

    def _fresh_x(self):
        if self.last_pose is None or (time.time() - self.last_pose_t) > self.pose_timeout_s:
            return None
        return float(self.last_pose.pose.position.x)

    def _call(self, client, name, wait_s=5.0):
        if not client.wait_for_service(timeout_sec=wait_s):
            self.get_logger().error('service %s недоступен' % name)
            return False, 'service unavailable'
        fut = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=wait_s)
        if fut.result() is None:
            return False, 'no response'
        return fut.result().success, fut.result().message

    def run(self) -> dict:
        self.get_logger().warn('Старт approach: /cup_approach_x/start')
        ok, msg = self._call(self.cli_start, '/cup_approach_x/start')
        if not ok:
            return {'ok': False, 'stage': 'approach_start', 'message': msg}

        t0 = time.time()
        reached = False
        while time.time() - t0 < self.timeout_s:
            rclpy.spin_once(self, timeout_sec=0.2)
            x = self._fresh_x()
            if x is not None and x <= self.stop_x_m:
                reached = True
                break
        if not reached:
            self._call(self.cli_stop, '/cup_approach_x/stop')
            return {'ok': False, 'stage': 'approach_timeout',
                    'message': 'не доехали за %.0fs (нет свежей позы или x не упал ниже stop_x)' % self.timeout_s}

        # approach-нода сама остановится по своему stop_x_m, но на всякий случай
        # дублируем — двойной stop безопасен (см. её же publish_stop идемпотентность)
        self._call(self.cli_stop, '/cup_approach_x/stop')
        self.get_logger().warn('На месте (x<=%.2f) — захват' % self.stop_x_m)
        time.sleep(0.5)  # дать роботу устаканиться после остановки перед захватом

        ok, msg = self._call(self.cli_grasp, '/g1_manip/grasp_cup', wait_s=15.0)
        if not ok:
            return {'ok': False, 'stage': 'grasp', 'message': msg}

        return {'ok': True, 'stage': 'done', 'message': msg}


def main():
    rclpy.init()
    n = CupFetchOrchestrator()
    result = n.run()
    n.get_logger().warn('РЕЗУЛЬТАТ: %s' % result)
    n.destroy_node()
    rclpy.shutdown()
    print(result)


if __name__ == '__main__':
    main()
