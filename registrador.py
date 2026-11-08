#!/usr/bin/env python3
"""Nó ROS 2 que registra a odometria publicada pelo Gazebo para todos os VANTs.

Assina /model/<nome>/odometry de cada VANT listado no routes.json e /missao/eventos,
e grava dois arquivos CSV:
  <prefixo>_odom.csv    t_wall, t_sim, drone, x, y, z    (no máximo 10 amostras/s por VANT)
  <prefixo>_eventos.csv t_wall, t_sim, drone, evento, wp

As posições são gravadas no referencial do mundo (corrigidas pela pose inicial).
Encerra sozinho ao receber o evento "fim" do multi_follower (ou com Ctrl+C).

Uso:  python3 registrador.py out/routes.json resultados/c1_r1
"""
import csv
import json
import os
import sys
import time
from functools import partial

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

PERIODO = 0.1   # s de simulação entre amostras gravadas de um mesmo VANT


class Registrador(Node):
    def __init__(self, cfg, prefixo):
        super().__init__('registrador')
        os.makedirs(os.path.dirname(prefixo) or '.', exist_ok=True)
        self.f_odom = open(prefixo + '_odom.csv', 'w', newline='')
        self.f_ev = open(prefixo + '_eventos.csv', 'w', newline='')
        self.w_odom = csv.writer(self.f_odom)
        self.w_ev = csv.writer(self.f_ev)
        self.w_odom.writerow(['t_wall', 't_sim', 'drone', 'x', 'y', 'z'])
        self.w_ev.writerow(['t_wall', 't_sim', 'drone', 'evento', 'wp'])
        self.spawn = {d['name']: d['spawn'] for d in cfg['drones']}
        self.offset = {}
        self.last = {}
        self.terminou = False
        self.n = 0
        for name in self.spawn:
            self.create_subscription(Odometry, f'/model/{name}/odometry', partial(self.on_odom, name), 50)
        self.create_subscription(String, '/missao/eventos', self.on_event, 100)
        self.get_logger().info(f'Registrando {len(self.spawn)} VANTs em {prefixo}_*.csv')

    def on_odom(self, name, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if name in self.last and t - self.last[name] < PERIODO - 1e-6:
            return
        self.last[name] = t
        p = msg.pose.pose.position
        if name not in self.offset:
            self.offset[name] = [s - o for s, o in zip(self.spawn[name], (p.x, p.y, p.z))]
        ox, oy, oz = self.offset[name]
        self.w_odom.writerow([f'{time.time():.3f}', f'{t:.3f}', name,
                              f'{p.x + ox:.4f}', f'{p.y + oy:.4f}', f'{p.z + oz:.4f}'])
        self.n += 1
        if self.n % 2000 == 0:
            self.f_odom.flush()

    def on_event(self, msg):
        e = json.loads(msg.data)
        self.w_ev.writerow([f'{time.time():.3f}', f"{e['t_sim']:.3f}", e['drone'], e['evento'], e['wp']])
        self.f_ev.flush()
        if e['evento'] == 'fim':
            self.terminou = True

    def close(self):
        self.f_odom.close()
        self.f_ev.close()


def main():
    if len(sys.argv) < 3:
        print('Uso: python3 registrador.py <routes.json> <prefixo_saida>')
        sys.exit(1)
    cfg = json.load(open(sys.argv[1]))
    rclpy.init()
    node = Registrador(cfg, sys.argv[2])
    try:
        while rclpy.ok() and not node.terminou:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.get_logger().info(f'{node.n} amostras gravadas.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
