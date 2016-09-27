#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import collections
import argparse

import numpy as np
import bs4

class Quaternion(collections.namedtuple(
        'Quaternion', 'x y z w')):

    @classmethod
    def make_from_angle(cls, x, y, z, angle):
        s = np.sin(angle / 2)
        return cls(x * s, y * s, z * s, np.cos(angle / 2))

    def __mul__(self, rhs):
        b, c, d, a = self
        f, g, h, e = rhs
        return Quaternion(
            b*e + a*f + c*h - d*g,
            a*g - b*h + c*e + d*f,
            a*h + b*g - c*f + d*e,
            a*e - b*f - c*g - d*h
        )


def main():
    parser = argparse.ArgumentParser(
        description='rotate quaternions of blocks in besiege saved file by '
        'given angle')
    parser.add_argument('input')
    parser.add_argument('-a', '--angle', type=float, default=3,
                        help='rotate angle in degress')
    parser.add_argument('output')
    args = parser.parse_args()

    with open(args.input) as fin:
        data = bs4.BeautifulSoup(fin, "xml")

    for i in data.select('Block[id=15]'):
        pos, = i.select('Position')
        pos = np.array(list(map(float, map(pos.__getitem__, 'xyz'))),
                       dtype=np.float)
        rot, = i.select('Rotation')
        q0 = Quaternion(*map(float, map(rot.__getitem__, 'xyzw')))
        q1_args = [0, 0, 0, np.deg2rad(args.angle)]
        idx = np.argmax(np.abs(pos))
        q1_args[idx] = 1
        if int(pos[idx] < 0) ^ int(idx == 2):
            q1_args[3] *= -1
        q = Quaternion.make_from_angle(*q1_args) * q0
        print(q0, '=>', q)

        for i in 'xyzw':
            rot[i] = getattr(q, i)

    with open(args.output, 'w') as fout:
        fout.write(str(data))

if __name__ == '__main__':
    main()
