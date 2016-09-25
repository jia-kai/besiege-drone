#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socketserver
import marshal
import socket

import numpy as np

HOST = 'localhost'
PORT = 2743

class State:
    """current flying state

    Note: Besiege applies a left-hand coordinate system:
        euler angle order: z, x, y
        euler angle direction: x:(z->y) y:(z->x) z:(y->x)

    We use a right-hand world coordinate
    """

    _prev_position = None

    engine_speed = None
    """current engine speed: (2, 2)

    :type: :class:`numpy.ndarray` (2, 2) corresponding to for
        ``[[fl, fr], [rl, rr]]``
    """

    time_delta = None
    """time for previous frame"""

    position = None
    """current (x, y, z) position"""

    position_speed = None

    top_dir = None
    """unit-vector describing top direction of top (i.e. propeller advancing
    direction) by applying the euler angles"""

    front_dir = None
    """unit-vector front direction"""

    right_dir = None
    """unit-vector right direction"""

    STARTING_TOP = np.array([0, 1, 0], dtype=np.float)
    STARTING_FRONT = np.array([0, 0, -1], dtype=np.float)
    STARTING_RIGHT = np.array([1, 0, 0], dtype=np.float)

    def __init__(self):
        self.engine_speed = np.ones((2, 2))

    def update(self, position, angle, time_delta):
        if time_delta == 0:
            return False
        self.position = np.array(position, dtype=np.float)
        self.position[2] *= -1
        angle = np.array(angle, dtype=np.float)
        self.time_delta = time_delta

        assert self.position.shape == (3, )
        angle.shape == (3, )
        assert isinstance(time_delta, float)

        if self._prev_position is not None:
            self.position_speed = (
                (self.position - self._prev_position) / time_delta)
        self._prev_position = self.position
        self._init_dir(angle)
        return True

    def clip(self):
        np.clip(self.engine_speed, 0.1, 1.9, self.engine_speed)

    def _init_dir(self, angles):
        def mkrot(axis, angle):
            c = np.cos
            s = np.sin
            mat = np.array(
                [[c(angle), s(angle)],
                 [-s(angle), c(angle)]]
            )
            ret = np.zeros((3, 3))
            a, b = [i for i in range(3) if i != axis]
            ret[[a, a, b, b], [a, b, a, b]] = mat.flatten()
            ret[axis, axis] = 1
            return ret
        rx, ry, rz = angles
        rot = mkrot(1, ry).dot(mkrot(0, rx)).dot(mkrot(2, -rz))
        self.top_dir = rot.dot(self.STARTING_TOP)
        self.front_dir = rot.dot(self.STARTING_FRONT)
        self.right_dir = rot.dot(self.STARTING_RIGHT)


class PIDController:
    _integ = 0
    _prev_err = None

    def __init__(self, state, kp, ki=0, kd=0, scale=1):
        self._state = state
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._scale = scale

    def __call__(self, err):
        ti = 0
        if self._ki:
            self._integ += err * self._state.time_delta
            ti = self._integ * self._ki

        td = 0
        if self._kd:
            diff = 0
            if self._prev_err is not None:
                diff = (err - self._prev_err) / self._state.time_delta
            self._prev_err = err
            td = diff * self._kd
        return (err * self._kp + ti + td) * self._scale


class GestureController:
    """controls yaw, pitch, roll_speed and altitude"""

    target_altitude_speed = 0
    target_top_dir = None

    def __init__(self, state):
        assert isinstance(state, State)
        self._state = state
        self._pid_altitude = PIDController(state, 1, 0, 1, scale=1e-3)
        self._pid_top = PIDController(state, 1, 0, 1, scale=1e-2)
        self.target_top_dir = np.array([0, 1, 0], dtype=np.float)

    def step(self):
        s = self._state
        if s.position_speed is None:
            return

        s.engine_speed += self._pid_altitude(
            self.target_altitude_speed - s.position_speed[1])
        self._adjust_top_dir()

    def _adjust_top_dir(self):
        """adjust top_dir towards target_top_dir"""
        s = self._state

        # (x, y) is the engine power distribution
        x = -self.target_top_dir.dot(s.right_dir)
        y = -self.target_top_dir.dot(s.front_dir)

        # solve a, b (standing for engine power distribution)
        #   (y+, front)
        #  a | b
        # ---+--- (x+, right)
        # -b | -a
        # so that total torque is (x, y)
        a = y - x
        b = y + x
        pdist = np.empty((2, 2), dtype=np.float)
        pdist[0] = a, b
        pdist[1] = -b, -a
        delta = self._pid_top(pdist)
        s.engine_speed += delta


class System:
    """hole controlling system"""

    def __init__(self):
        self._state = State()
        self.gesture = GestureController(self._state)

    def step(self, *args):
        if self._state.update(*args):
            self.gesture.step()
        self._state.clip()
        return list(map(float, self._state.engine_speed.flatten()))


class RPCHandler(socketserver.BaseRequestHandler):
    def handle(self):
        system = System()
        while True:
            dlen = int(self.request.recv(32))
            data = b''
            while len(data) < dlen:
                data += self.request.recv(1024)
            assert len(data) == dlen
            args = marshal.loads(data)
            ret = system.step(*args)
            self.request.sendall(
                (';'.join(map(str, ret)) + '@').encode('ascii'))

class RPCHandlerServer(socketserver.TCPServer):
    allow_reuse_address = True

def main():
    server = RPCHandlerServer((HOST, PORT), RPCHandler)
    server.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    server.serve_forever()

if __name__ == '__main__':
    main()
