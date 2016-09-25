#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from actiondef import Action

import socketserver
import marshal
import socket
from abc import ABCMeta, abstractmethod, abstractproperty

import numpy as np

HOST = 'localhost'
PORT = 2743

def rot_by_angle(vec, ref, angle):
    """Rotate *vec* by *angle* radians on the plan perpendicular to *ref*"""
    ax = np.cross(ref, vec)
    return np.cos(angle) * vec + np.sin(angle) * ax

def norm_angle_diff(v):
    if abs(v) > np.pi:
        if v > 0:
            v -= np.pi * 2
        else:
            v += np.pi * 2
        assert abs(v) <= np.pi
    return v

class State:
    """current flying state

    Note: Besiege applies a left-hand coordinate system:
        euler angle order: z, x, y
        euler angle direction: x:(y->z) y:(z->x) z:(x->y)

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

    yaw = None
    """yaw angle in radians"""

    _prev_yaw = None

    yaw_speed = None

    command = None
    """current user command: OR of actions"""

    STARTING_TOP = np.array([0, 1, 0], dtype=np.float)
    STARTING_FRONT = np.array([0, 0, -1], dtype=np.float)
    STARTING_RIGHT = np.array([1, 0, 0], dtype=np.float)

    def __init__(self):
        self.engine_speed = np.ones((2, 2))

    def update(self, position, angle, command, time_delta):
        self.command = command
        if time_delta == 0:
            return False
        self.position = np.array(position, dtype=np.float)
        self.position[2] *= -1
        angle = np.array(angle, dtype=np.float)
        self.time_delta = time_delta

        assert self.position.shape == (3, )
        angle.shape == (3, )
        assert isinstance(time_delta, float)

        self._init_dir(angle)

        if self._prev_position is not None:
            self.position_speed = (
                (self.position - self._prev_position) / time_delta)
            self.yaw_speed = (
                norm_angle_diff(self.yaw - self._prev_yaw) / time_delta)
            ret = True
        else:
            ret = False
        self._prev_position = self.position
        self._prev_yaw = self.yaw
        return ret

    def clip(self):
        np.clip(self.engine_speed, 0.1, 1.9, self.engine_speed)

    def _init_dir(self, angles):
        def mkrot(axis, angle, swap=False):
            c = np.cos
            s = np.sin
            mat = np.array(
                [[c(angle), -s(angle)],
                 [s(angle), c(angle)]]
            )
            ret = np.zeros((3, 3))
            a, b = [i for i in range(3) if i != axis]
            mf = mat.flatten()
            if swap:
                mf = mf[::-1]
            ret[[a, a, b, b], [a, b, a, b]] = mf
            ret[axis, axis] = 1
            return ret
        rx, ry, rz = angles
        rot = mkrot(1, -ry, True).dot(mkrot(0, rx, True)).dot(mkrot(2, rz))
        self.top_dir = rot.dot(self.STARTING_TOP)
        self.front_dir = rot.dot(self.STARTING_FRONT)
        self.right_dir = rot.dot(self.STARTING_RIGHT)
        x, y, z = self.front_dir
        self.yaw = np.arctan2(x, z)


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


class HoverStabilizerBase(metaclass=ABCMeta):
    IDLE_TIME_THRESH = 1
    """use value after no user command for this period as target hover state"""

    _idle_time = 0

    _target_state = None

    @abstractproperty
    def USER_COMMAND_MASK(self):
        """mask for related user command"""

    @abstractproperty
    def _state(self):
        """global state object

        :type: :class:`State`
        """

    @abstractproperty
    def _pid(self):
        """the pid controller"""

    @abstractmethod
    def get_state(self):
        """get current state for the signal to be controlled"""

    @abstractmethod
    def _apply_action(self, action):
        """apply action from PID controller"""

    def step(self):
        if self._state.command & self.USER_COMMAND_MASK:
            self._idle_time = 0
            return
        prev_idle_time = self._idle_time
        self._idle_time += self._state.time_delta
        if prev_idle_time < self.IDLE_TIME_THRESH:
            if self._idle_time >= self.IDLE_TIME_THRESH:
                # first time to cross IDLE_TIME_THRESH, so use current state as
                # target state
                self._target_state = self.get_state()
                print('{}: target={}'.format(type(self).__name__,
                                             self._target_state))
            return

        self._apply_action(self._pid(
            self._err(self._target_state, self.get_state())))

    def _err(self, target, cur):
        return target - cur


class PIDOnGestureHoverStabilizer(HoverStabilizerBase):
    @abstractproperty
    def _pid_args(self):
        pass

    _state = None
    _pid = None

    def __init__(self, state, gesture_controller):
        assert isinstance(state, State)
        assert isinstance(gesture_controller, GestureController)
        self._state = state
        self._gesture_controller = gesture_controller
        self._pid = PIDController(state, *self._pid_args)


class AltitudeHoverStabilizer(PIDOnGestureHoverStabilizer):
    IDLE_TIME_THRESH = 0.5
    _pid_args = (2.5, )
    USER_COMMAND_MASK = Action.HIGHER | Action.LOWER

    def get_state(self):
        return self._state.position[1]

    def _apply_action(self, action):
        self._gesture_controller.target_altitude_speed = action


class YawHoverStabilizer(PIDOnGestureHoverStabilizer):
    _pid_args = (2.5, )
    USER_COMMAND_MASK = Action.YAW_LEFT | Action.YAW_RIGHT

    def get_state(self):
        return self._state.yaw

    def _apply_action(self, action):
        self._gesture_controller.target_yaw_speed = action

    def _err(self, target, cur):
        return norm_angle_diff(target - cur)


class PositionHoverStabilizer(PIDOnGestureHoverStabilizer):
    IDLE_TIME_THRESH = 3
    _pid_args = (1, 0, 1, 0.01)

    USER_COMMAND_MASK = Action.GO_FRONT | Action.GO_BACK

    def get_state(self):
        return self._state.position[[0, 2]]

    def _apply_action(self, action):
        self._gesture_controller.target_top_dir = action


class GestureController:
    """controls yaw, pitch, roll_speed and altitude"""

    target_altitude_speed = 0
    _target_top_dir = None
    target_yaw_speed = 0

    MAX_TOP_ANGLE_DEG = 10
    _MAX_TOP_ANGLE_TAN = np.tan(np.deg2rad(MAX_TOP_ANGLE_DEG))

    SPEED_LIMIT = 10

    @property
    def target_top_dir(self):
        return self._target_top_dir

    @target_top_dir.setter
    def target_top_dir(self, v):
        v = np.array(v, dtype=np.float)
        if v.size == 2:
            mag = np.linalg.norm(v)
            if mag > self._MAX_TOP_ANGLE_TAN:
                v *= self._MAX_TOP_ANGLE_TAN / mag
            dx, dz = v
            v = np.array([dx, 1, dz], dtype=np.float)
        assert v.shape == (3, )
        v /= np.linalg.norm(v)
        self._target_top_dir = v

    def __init__(self, state):
        assert isinstance(state, State)
        self._state = state
        self._pid_altitude = PIDController(state, 1, 0, 1, scale=1e-3)
        self._pid_top = PIDController(state, 1, 0, 1, scale=1e-2)
        self._pid_yaw = PIDController(state, 1, 0, 1, scale=1e-2)
        self._target_top_dir = np.array([0, 1, 0], dtype=np.float)

    def step(self):
        self._adjust_altitude()
        self._adjust_top_dir()
        self._adjust_yaw()

    def setup_target(self):
        cmd = self._state.command

        pitch = 0
        if cmd & Action.GO_FRONT:
            pitch += np.deg2rad(10)
        if cmd & Action.GO_BACK:
            pitch -= np.deg2rad(10)
        speed = np.linalg.norm(self._state.position_speed[[0, 2]])
        if speed >= self.SPEED_LIMIT:
            pitch *= (self.SPEED_LIMIT * 2 - speed) / self.SPEED_LIMIT
        self.target_top_dir = rot_by_angle(
            State.STARTING_TOP,
            np.cross(State.STARTING_TOP, self._state.front_dir),
            pitch)

        self.target_yaw_speed = 0
        if cmd & Action.YAW_LEFT:
            self.target_yaw_speed += np.deg2rad(45)
        if cmd & Action.YAW_RIGHT:
            self.target_yaw_speed -= np.deg2rad(45)

        self.target_altitude_speed = 0
        if cmd & Action.HIGHER:
            self.target_altitude_speed += 30
        if cmd & Action.LOWER:
            self.target_altitude_speed -= 10

    def _adjust_altitude(self):
        s = self._state
        s.engine_speed += self._pid_altitude(
            self.target_altitude_speed - s.position_speed[1])

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

    def _adjust_yaw(self):
        s = self._state
        a = self._pid_yaw(self.target_yaw_speed - s.yaw_speed)
        s.engine_speed += [[-a, a],
                           [a, -a]]


class System:
    """hole controlling system"""

    def __init__(self):
        self._state = State()
        self.gesture = GestureController(self._state)
        self._stabilizers = [
            i(self._state, self.gesture) for i in [
                AltitudeHoverStabilizer,
                YawHoverStabilizer,
                PositionHoverStabilizer
            ]]

    def step(self, *args):
        if self._state.update(*args):
            self.gesture.setup_target()
            for i in self._stabilizers:
                i.step()
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
