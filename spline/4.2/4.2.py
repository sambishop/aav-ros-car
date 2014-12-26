#!/usr/bin/python

from math import sqrt
from sys import stderr

wp = [(0.0, 0.0), (5.0, 5.0), (7.0, 2.0), (4.0, 1.0)]

def gen_S(P0, P5, s_d1, e_d1):
    s_d2 = 0.0
    e_d2 = 0.0
    P1 = 1.0 / 5.0 * s_d1 + P0
    P2 = 1.0 / 20.0 * s_d2 + 2 * P1 - P0
    P4 = P5 - 1.0 / 5.0 * e_d1
    P3 = 1.0 / 20.0 * e_d2 + 2 * P4 - P5
    def S(u):
        return (1.0 - u) ** 5 * P0 \
            + 5.0 * (1.0 - u) ** 4 * u * P1 \
            + 10.0 * (1.0 - u) ** 3 * u ** 2 * P2 \
            + 10.0 * (1.0 - u) ** 2 * u ** 3 * P3 \
            + 5.0 * (1.0 - u) * u ** 4 * P4 \
            + u ** 5 * P5
    return S

def length(v):
    return sqrt(v[0] ** 2 + v[1] ** 2)

def normalize(v):
    magnitude = length(v)
    return (v[0] / magnitude, v[1] / magnitude)

dxs = [(wp[len(wp) - 1][0] - wp[len(wp) - 2][0]) / 2.0]
dys = [(wp[len(wp) - 1][1] - wp[len(wp) - 2][1]) / 2.0]
for i in range(len(wp) - 2, 0, -1):
    p0 = wp[i - 1]
    p1 = wp[i]
    p2 = wp[i + 1]
    v0 = (p1[0] - p0[0], p1[1] - p0[1])
    v1 = (p2[0] - p1[0], p2[1] - p1[1])
    print >> stderr, 'v0: %s, v1: %s' % (v0, v1)
    m0 = length(v0)
    m1 = length(v1)
    m = m0 / 2.0 if m0 < m1 else m1 / 2.0
    print >> stderr, '%s and %s, chose %s' % (m0, m1, m)
    v0 = normalize(v0)
    v1 = normalize(v1)
    print >> stderr, "v0': %s, v1': %s" % (v0, v1)
    s = ((v0[0] + v1[0]) / 2.0 * m, (v0[1] + v1[1]) / 2.0 * m)
    print >> stderr, 's = %s (slope = %f)' % (s, s[1] / s[0])
    dxs.insert(0, s[0])
    dys.insert(0, s[1])
dxs.insert(0, (wp[1][0] - wp[0][0]) / 2.0)
dys.insert(0, (wp[1][1] - wp[0][1]) / 2.0)
print >> stderr, 'dxs = %s' % dxs
print >> stderr, 'dys = %s' % dys

for i in range(0, len(wp) - 1):
    p0 = wp[i]
    p1 = wp[i + 1]
    Sx = gen_S(p0[0], p1[0], dxs[i], dxs[i + 1])
    Sy = gen_S(p0[1], p1[1], dys[i], dys[i + 1])
    for u in [i * .01 for i in range(0, 100)]:
        p = (Sx(u), Sy(u))
        print p[0], p[1]

