#!/usr/bin/python

from math import atan2, cos, sin, sqrt
from sys import stderr

def gen_fx(p0, p1):
    d = sqrt((p1[0] - p0[0]) ** 2 + (p1[1] - p0[1]) ** 2)
    def fx(u):
        return u * d
    return fx

def fy(u):
    return 0.0

def gen_rot(theta):
    c = cos(theta)
    s = sin(theta)
    def rot(p):
        return (p[0] * c - p[1] * s, p[0] * s - p[1] * c)
    return rot

def gen_tran(p0):
    def tran(p):
        return (p0[0] + p[0], p0[1] + p[1])
    return tran

wp = [(0.0, 0.0), (5.0, 5.0), (7.0, 2.0), (4.0, 1.0)]
for i in range(0, len(wp) - 1):
    p0 = wp[i]
    p1 = wp[i + 1]

    theta = atan2(p1[1] - p0[1], p1[0] - p0[0])

    fx = gen_fx(p0, p1)
    rot = gen_rot(theta)
    tran = gen_tran(p0)

    for u in [i * .01 for i in range(0, 100 + 1)]:
        x = fx(u)
        y = fy(u)
        p = rot((fx(u), fy(u)))
        p = tran(p)
        print p[0], p[1]

