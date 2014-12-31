#!/usr/bin/python

from math import sqrt
from sys import stderr

wp = [(0.0, 0.0), (5.0, 5.0), (7.0, 2.0), (4.0, 1.0)]

def plot(f1=['plot']):
    f2 = f1[0]
    f1[0] = 'replot'
    return f2

def length(v):
    return sqrt(v[0] ** 2 + v[1] ** 2)

def normalize(v):
    magnitude = length(v)
    return (v[0] / magnitude, v[1] / magnitude)

d1s = [((wp[len(wp) - 1][0] - wp[len(wp) - 2][0]) / 2.0,
    (wp[len(wp) - 1][1] - wp[len(wp) - 2][1]) / 2.0)]
for i in range(len(wp) - 2, 0, -1):
    p0 = wp[i - 1]
    p1 = wp[i]
    p2 = wp[i + 1]
    v0 = (p1[0] - p0[0], p1[1] - p0[1])
    v1 = (p2[0] - p1[0], p2[1] - p1[1])
    m0 = length(v0)
    m1 = length(v1)
    m = m0 if m0 < m1 else m1
    v0 = normalize(v0)
    v1 = normalize(v1)
    d1 = ((v0[0] + v1[0]) / 2.0 * m, (v0[1] + v1[1]) / 2.0 * m)
    d1s.insert(0, d1)
d1s.insert(0, ((wp[1][0] - wp[0][0]) / 2.0, (wp[1][1] - wp[0][1]) / 2.0))
print >> stderr, d1s

print '''set key off

set xrange [-1:8]
set yrange [-1:6]
set terminal wxt size 440,345

set parametric
set trange [0:1]

S(P0, P1, P2, P3, P4, P5, t) = (1.0 - t) ** 5 * P0 \\
        + 5.0 * (1.0 - t) ** 4 * t * P1 \\
        + 10.0 * (1.0 - t) ** 3 * t ** 2 * P2 \\
        + 10.0 * (1.0 - t) ** 2 * t ** 3 * P3 \\
        + 5.0 * (1.0 - t) * t ** 4 * P4 \\
        + t ** 5 * P5
f(a, b, t) = a + t * (b - a)
'''

for i in range(0, len(wp) - 1):
    p0 = wp[i]
    p1 = wp[i + 1]
    print plot() + ' f(%d, %d, t) lt 4, f(%d, %d, t) lt 4' \
            % (p0[0], p1[0], p0[1], p1[1])

for i in range(0, len(wp) - 1):
    p0 = wp[i]
    p1 = wp[i + 1]
    print plot(),
    for axis in [0, 1]:
        P0 = p0[axis]
        P5 = p1[axis]
        s_d1 = d1s[i][axis]
        e_d1 = d1s[i + 1][axis]
        s_d2 = 0.0
        e_d2 = 0.0
        P1 = 1.0 / 5.0 * s_d1 + P0
        P2 = 1.0 / 20.0 * s_d2 + 2 * P1 - P0
        P4 = P5 - 1.0 / 5.0 * e_d1
        P3 = 1.0 / 20.0 * e_d2 + 2 * P4 - P5
        print 'S(' + str(P0) + ', ' \
                + str(P1) + ', ' \
                + str(P2) + ', ' \
                + str(P3) + ', ' \
                + str(P4) + ', ' \
                + str(P5) + ', t) lt -1' \
                + (',' if axis == 0 else ''),
    print ''
print ''

