"""Lógica de seguimento de rota (sem dependência de ROS, para poder ser testada isolada)."""
import math


class RouteFollower:
    """Controlador do tipo "ir até o ponto" para um VANT.

    waypoints : lista [(x, y), ...] no sistema de coordenadas da simulação
    spawn     : (x, y, z) onde o VANT nasce no mundo
    speed     : módulo da velocidade horizontal (m/s, escala da simulação)
    altitude  : altitude de voo constante (m)
    tolerance : distância (m) para considerar o ponto de passagem alcançado
    stops     : lista de paradas de recarga [{"wp": índice, "dur_sim": segundos}, ...]

    A cada chamada de step() o controlador:
      1. converte a odometria para o referencial do mundo;
      2. avança o índice do ponto-alvo enquanto o VANT estiver dentro da tolerância;
      3. se o ponto alcançado for uma estação de recarga, mantém o VANT parado
         (pairando na própria altitude) pelo tempo de recarga;
      4. caso contrário, comanda velocidade constante na direção do ponto-alvo.
    """

    def __init__(self, waypoints, spawn, speed, altitude, tolerance, stops=None):
        self.wp = [tuple(w) for w in waypoints]
        self.spawn = tuple(spawn)
        self.speed = speed
        self.alt = altitude
        self.tol = tolerance
        self.stops = {s["wp"]: s["dur_sim"] for s in (stops or [])}
        self.i = 0
        self.offset = None
        self.done = False
        self.hold_until = None     # instante (tempo de simulação) em que a recarga termina
        self.events = []           # eventos gerados no último step()

    def set_reference(self, odom):
        # Funciona tanto se a odometria vier no referencial do mundo quanto
        # se vier relativa à pose inicial: o offset é calculado na 1ª leitura.
        self.offset = tuple(s - o for s, o in zip(self.spawn, odom))

    def world_position(self, odom):
        return tuple(o + f for o, f in zip(odom, self.offset))

    def step(self, odom, t=0.0):
        """odom = (x, y, z) lido da odometria; t = tempo de simulação (s). Retorna (vx, vy, vz)."""
        self.events = []
        if self.offset is None:
            self.set_reference(odom)
        x, y, z = self.world_position(odom)
        vz = max(-self.speed, min(self.speed, self.alt - z))

        if self.hold_until is not None:                     # recarregando
            if t < self.hold_until:
                return 0.0, 0.0, vz
            self.hold_until = None
            self.events.append(("fim_recarga", self.i - 1))

        while self.i < len(self.wp):
            tx, ty = self.wp[self.i]
            if math.hypot(tx - x, ty - y) >= self.tol:
                break
            reached = self.i
            self.i += 1
            self.events.append(("waypoint", reached))
            if reached in self.stops and self.stops[reached] > 0:
                self.hold_until = t + self.stops[reached]
                self.events.append(("inicio_recarga", reached))
                return 0.0, 0.0, vz

        if self.i >= len(self.wp):
            if not self.done:
                self.events.append(("concluido", self.i))
            self.done = True
            return 0.0, 0.0, vz

        tx, ty = self.wp[self.i]
        d = math.hypot(tx - x, ty - y)
        return (tx - x) / d * self.speed, (ty - y) / d * self.speed, vz
