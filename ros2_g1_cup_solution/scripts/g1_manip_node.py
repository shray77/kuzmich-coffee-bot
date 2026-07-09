#!/usr/bin/env python3
"""
g1_manip_node.py — реальный сервис захвата чашки, закрывает пробел "fake /g1_manip"
из плана восстановления (см. пункт 3 "Следующие этапы" в отчёте).

Не трогает cup_center_then_adaptive_node.py и остальной перцепшн-контур —
отдельный ROS 2 node с двумя Trigger-сервисами, тем же паттерном что
/cup_approach_x/start и /cup_approach_x/stop:

  /g1_manip/grasp_cup    — закрыть кисть с контролем силы (не раздавить чашку)
  /g1_manip/release_cup  — открыть кисть

Управление кистью — HandClient из unitree_sdk2py.g1.hand.hand_client, тот же
API что в force_sensor.py (репо burunov-joke-bot), сюда перенесён самодостаточным
файлом, чтобы deploy_to_home.sh мог просто скопировать его в ~/ как остальные
скрипты пакета, без зависимости от другого репозитория.

⚠️ Собрано под Inspire RH56DFTP (см. параметр hand_type). Если на роботе
реально другая кисть (Dex3 — топики rt/dex3/<side>/cmd, см. официальную
доку dds_services.json — или BrainCo — топики rt/brainco/<side>/cmd) —
HandClient.Init() ниже упадёт или GetHandForce/SetHandAngle не найдутся:
смотри лог, там будет explicit "MISSING" причина, а не тихий сбой.

Использование (как и остальные node пакета):
  python3 ~/g1_manip_node.py --ros-args -p iface:=eth0 -p hand_used:=right \
    -p target_force_g:=80.0 -p max_force_g:=1200.0 -p dry_run:=false

Проверка:
  ros2 service call /g1_manip/grasp_cup std_srvs/srv/Trigger {}
  ros2 service call /g1_manip/release_cup std_srvs/srv/Trigger {}
"""
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

# ─── Калибровка силы (граммы, сенсор Inspire RH56DFTP: 10..2500) ───────────
FORCE_EMPTY, FORCE_TOUCH = 10, 30
FORCE_GRIP_LIGHT, FORCE_GRIP_FIRM, FORCE_TOO_HARD = 80, 350, 1200
CLOSE_STEP_RAD, SETTLE_TIME_S = 0.1, 0.05
STABILITY_WINDOW, STABILITY_TOL_G = 3, 15
MAX_CLOSE_STEPS = 30


class G1ManipNode(Node):
    def __init__(self):
        super().__init__('g1_manip_node')
        d = {'iface': 'eth0', 'hand_used': 'right', 'hand_type': 'RH56DFTP',
             'target_force_g': float(FORCE_GRIP_LIGHT), 'max_force_g': float(FORCE_TOO_HARD),
             'dry_run': False}
        for k, v in d.items():
            self.declare_parameter(k, v)
        for k in d:
            setattr(self, k, self.get_parameter(k).value)
        self.target_force_g = float(self.target_force_g)
        self.max_force_g = float(self.max_force_g)

        self._client = None
        self._ready = False
        if not self.dry_run:
            self._ready = self._init_hand()
        else:
            self.get_logger().warn('DRY RUN: HandClient не инициализируется')

        self.create_service(Trigger, '/g1_manip/grasp_cup', self.grasp_cb)
        self.create_service(Trigger, '/g1_manip/release_cup', self.release_cb)
        self.get_logger().warn(
            'g1_manip_node ready: hand=%s type=%s ready=%s target=%.0fg max=%.0fg' %
            (self.hand_used, self.hand_type, self._ready, self.target_force_g, self.max_force_g))

    def _init_hand(self) -> bool:
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            ChannelFactoryInitialize(0, self.iface)
        except Exception as e:
            self.get_logger().error('ChannelFactoryInitialize failed: %r' % e)
            return False

        if self.hand_type != 'RH56DFTP':
            self.get_logger().error(
                'hand_type=%s не реализован в этом node (только RH56DFTP). '
                'Если рука на самом деле Dex3 или BrainCo — нужен другой клиент, '
                'см. docstring вверху файла.' % self.hand_type)
            return False

        try:
            from unitree_sdk2py.g1.hand.hand_client import HandClient
            self._client = HandClient()
            self._client.Init()
            self._client.SetTimeout(10.0)
            self.get_logger().warn('SUCCESS: HandClient initialized (RH56DFTP, iface=%s)' % self.iface)
            return True
        except Exception as e:
            self.get_logger().error('HandClient init failed: %r' % e)
            self.get_logger().error(
                'Возможные причины: SDK не установлен / рука физически не подключена / '
                'неверный iface / это не RH56DFTP')
            return False

    # ─── низкоуровневые команды кисти ──────────────────────────────────────
    def _set_angles(self, angles):
        if self.dry_run or not self._ready:
            self.get_logger().info('DRY set_angles(%s, %s)' % (self.hand_used, angles))
            return True
        try:
            return self._client.SetHandAngle(self.hand_used, angles) == 0
        except AttributeError:
            try:
                return self._client.SetHandPose(self.hand_used, angles) == 0
            except Exception as e:
                self.get_logger().error('_set_angles failed: %r' % e)
                return False
        except Exception as e:
            self.get_logger().error('_set_angles failed: %r' % e)
            return False

    def _close_rad(self, rad):
        rad = max(0.0, min(1.5, rad))
        return self._set_angles([rad, rad, rad, rad, rad, 0.0])

    def _open(self):
        return self._set_angles([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def _relax(self):
        return self._set_angles([0.3, 0.3, 0.3, 0.3, 0.3, 0.0])

    def _read_force_g(self) -> float:
        """Максимум усилия по обеим кистям в граммах. dry_run/не готов -> 0."""
        if self.dry_run or not self._ready:
            return 0.0
        try:
            force = self._client.GetHandForce(self.hand_used)
            return float(max(force)) if isinstance(force, (list, tuple)) else float(force)
        except AttributeError:
            pass
        except Exception as e:
            self.get_logger().debug('GetHandForce failed: %r' % e)
        try:
            state = self._client.GetHandState(self.hand_used)
            if hasattr(state, 'force'):
                return float(state.force)
            if hasattr(state, 'force_g'):
                return float(state.force_g)
        except Exception as e:
            self.get_logger().debug('GetHandState failed: %r' % e)
        return 0.0

    # ─── сервисы ────────────────────────────────────────────────────────────
    def grasp_cb(self, req, resp):
        self.get_logger().warn('GRASP requested: hand=%s target=%.0fg max=%.0fg' %
                                (self.hand_used, self.target_force_g, self.max_force_g))
        self._open()
        time.sleep(0.3)

        current_rad, stability_count, last_force = 0.0, 0, 0.0
        for step in range(MAX_CLOSE_STEPS):
            current_rad = min(current_rad + CLOSE_STEP_RAD, 1.5)
            self._close_rad(current_rad)
            time.sleep(SETTLE_TIME_S)
            force = self._read_force_g()
            self.get_logger().info('grasp step=%d rad=%.2f force=%.1fg' % (step, current_rad, force),
                                    throttle_duration_sec=0.5)

            if force >= self.max_force_g:
                self._relax()
                resp.success = False
                resp.message = 'Перетянули: %.0fг > max %.0fг. Расслабил кисть.' % (force, self.max_force_g)
                self.get_logger().error(resp.message)
                return resp

            if force >= self.target_force_g:
                stability_count = stability_count + 1 if abs(force - last_force) < STABILITY_TOL_G else 0
                last_force = force
                if stability_count >= STABILITY_WINDOW:
                    resp.success = True
                    resp.message = 'Взял. Сила %.0fг после %d шагов.' % (force, step + 1)
                    self.get_logger().warn(resp.message)
                    return resp
            last_force = force

        resp.success = False
        resp.message = 'Не взял за %d шагов, финальная сила %.0fг — чашки нет в захвате?' % (
            MAX_CLOSE_STEPS, last_force)
        self.get_logger().error(resp.message)
        return resp

    def release_cb(self, req, resp):
        self._open()
        time.sleep(0.3)
        resp.success = True
        resp.message = 'Кисть открыта'
        self.get_logger().warn(resp.message)
        return resp


def main():
    rclpy.init()
    n = G1ManipNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            n._relax()
        except Exception:
            pass
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
