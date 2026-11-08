#!/usr/bin/env python3
"""Nó ROS 2 que comanda todos os VANTs ao mesmo tempo.

Para cada VANT:
  - assina /model/<nome>/odometry   (nav_msgs/Odometry, vindo do Gazebo pela ponte)
  - publica /model/<nome>/cmd_vel   (geometry_msgs/Twist, para o plugin VelocityControl)
Além disso publica os eventos da missão em /missao/eventos (std_msgs/String, JSON),
que o registrador grava junto com a odometria.

O laço de controle roda a 10 Hz. O tempo usado nas paradas de recarga é o tempo de
simulação lido do carimbo de tempo da odometria.

Uso:  python3 multi_follower.py out/routes.json
"""
import json
import sys
from functools import partial

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from follower_logic import RouteFollower

DT = 0.1  # período do laço de controle (s) -> 10 Hz


class MultiFollower(Node):
    def __init__(self, cfg):
        super().__init__('multi_follower')
        self.drones = {}
        self.ev_pub = self.create_publisher(String, '/missao/eventos', 50)
        for d in cfg['drones']:
            name = d['name']
            self.drones[name] = {
                'f': RouteFollower(d['waypoints'], d['spawn'], d['speed'], d['altitude'],
                                   d['tolerance'], d.get('stops', [])),
                'pub': self.create_publisher(Twist, f'/model/{name}/cmd_vel', 10),
                'odom': None, 't': 0.0, 'stopped': False,
            }
            self.create_subscription(Odometry, f'/model/{name}/odometry', partial(self.on_odom, name), 10)
            self.get_logger().info(
                f"{name}: {len(d['waypoints'])} waypoints, {len(d.get('stops', []))} recargas, "
                f"{d['speed']:.2f} m/s, altitude {d['altitude']:.1f} m")
        self.started = False
        self.finished = False
        self.timer = self.create_timer(DT, self.loop)

    def on_odom(self, name, msg):
        p = msg.pose.pose.position
        self.drones[name]['odom'] = (p.x, p.y, p.z)
        self.drones[name]['t'] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def event(self, name, kind, idx, t):
        msg = String()
        msg.data = json.dumps({"drone": name, "evento": kind, "wp": idx, "t_sim": t})
        self.ev_pub.publish(msg)

    def loop(self):
        if not self.started:
            if not all(d['odom'] is not None for d in self.drones.values()):
                return  # espera todos aparecerem (e o play do Gazebo)
            self.started = True
            self.get_logger().info('Odometria de todos os VANTs recebida. Iniciando a missão.')
            for name, d in self.drones.items():
                self.event(name, 'inicio', 0, d['t'])

        for name, d in self.drones.items():
            if d['stopped']:
                continue
            f = d['f']
            vx, vy, vz = f.step(d['odom'], d['t'])
            cmd = Twist()
            cmd.linear.x, cmd.linear.y, cmd.linear.z = vx, vy, vz
            d['pub'].publish(cmd)
            for kind, idx in f.events:
                self.event(name, kind, idx, d['t'])
                if kind == 'inicio_recarga':
                    self.get_logger().info(f"{name}: recarregando no ponto {idx} "
                                           f"({f.stops[idx]:.0f} s de simulação)")
                elif kind == 'waypoint' and (idx % 5 == 0):
                    self.get_logger().info(f"{name}: waypoint {idx + 1}/{len(f.wp)}")
            if f.done:
                d['pub'].publish(Twist())
                d['stopped'] = True
                self.get_logger().info(f'{name}: missão concluída (t_sim = {d["t"]:.1f} s)')

        if all(d['stopped'] for d in self.drones.values()) and not self.finished:
            self.finished = True
            self.event('todos', 'fim', -1, max(d['t'] for d in self.drones.values()))
            self.get_logger().info('Todos os VANTs concluíram a missão.')
            self.timer.cancel()
            self.create_timer(1.0, self.quit)   # dá tempo do evento chegar ao registrador

    def quit(self):
        raise SystemExit


def main():
    if len(sys.argv) < 2:
        print('Uso: python3 multi_follower.py <routes.json>')
        sys.exit(1)
    cfg = json.load(open(sys.argv[1]))
    rclpy.init()
    node = MultiFollower(cfg)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
