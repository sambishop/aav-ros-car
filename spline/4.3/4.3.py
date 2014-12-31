#!/usr/bin/python

from math import sqrt
from sys import stderr

wp = [(0.0, 0.0), (2.0, 4.0), (5.0, 2.0)]

def length(v):
    return sqrt(v[0] ** 2 + v[1] ** 2)

def normalize(v):
    magnitude = length(v)
    return (v[0] / magnitude, v[1] / magnitude)

# d1s
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
    #m = m0 / 2.0 if m0 < m1 else m1 / 2.0
    m = m0 if m0 < m1 else m1
    v0 = normalize(v0)
    v1 = normalize(v1)
    d1 = ((v0[0] + v1[0]) / 2.0 * m, (v0[1] + v1[1]) / 2.0 * m)
    d1s.insert(0, d1)
d1s.insert(0, ((wp[1][0] - wp[0][0]) / 2.0, (wp[1][1] - wp[0][1]) / 2.0))
print >> stderr, d1s

# d2s
d2s = [[], []]
for axis in [0, 1]:
    P0 = wp[0][axis]
    P3 = wp[1][axis]
    P1 = P0 + (1.0 / 3.0) * d1s[0][axis]
    P2 = P3 - (1.0 / 3.0) * d1s[1][axis]
    d2s[axis].insert(0, 6.0 * (P0 - 2.0 * P1 + P2))
for i in range(len(wp) - 2, 0, -1):
    A = wp[i - 1]
    B = wp[i]
    C = wp[i + 1]
    v0 = (B[0] - A[0], B[1] - A[1])
    v1 = (C[0] - B[0], C[1] - B[1])
    m0 = length(v0)
    m1 = length(v1)
    alpha = m1 / (m0 + m1)
    beta = m0 / (m0 + m1)
    a0 = alpha * (6.0 * A[0] + 2.0 * d1s[i - 1][0] + 4.0 * d1s[i][0] - 6 * B[0]) + beta * (-6.0 * B[0] - 4.0 * d1s[i][0] - 2.0 * d1s[i + 1][0] + 6 * C[0])
    a1 = alpha * (6.0 * A[1] + 2.0 * d1s[i - 1][1] + 4.0 * d1s[i][1] - 6 * B[1]) + beta * (-6.0 * B[1] - 4.0 * d1s[i][1] - 2.0 * d1s[i + 1][1] + 6 * C[1])
    d2s[0].insert(1, a0)
    d2s[1].insert(1, a1)
for axis in [0, 1]:
    A = wp[1][axis]
    B = wp[2][axis]
    tA = d1s[1][axis]
    tB = d1s[2][axis]
    d2s[axis].append(6 * A + 2 * tA + 4 * tB - 6 * B)
print >> stderr, d2s

print '''set key off

#set xrange [1:3]
#set yrange [3:5]
set xrange [-1:6]
set yrange [-1:6]
set terminal wxt size 350,345

set parametric
set trange [0:1]

f(a, b, t) = a + t * (b - a)
S3(P0, P1, P2, P3, t) = (1.0 - t) ** 3 * P0 \\
        + 3.0 * t * (1.0 - t) ** 2 * P1 \\
        + 3.0 * t ** 2 * (1.0 - t) * P2 \\
        + t ** 3 * P3
S5(P0, P1, P2, P3, P4, P5, t) = (1.0 - t) ** 5 * P0 \\
        + 5.0 * (1.0 - t) ** 4 * t * P1 \\
        + 10.0 * (1.0 - t) ** 3 * t ** 2 * P2 \\
        + 10.0 * (1.0 - t) ** 2 * t ** 3 * P3 \\
        + 5.0 * (1.0 - t) * t ** 4 * P4 \\
        + t ** 5 * P5

plot \\'''

for i in range(0, len(wp) - 1):
    p0 = wp[i]
    p1 = wp[i + 1]
    print ' f(%d, %d, t) lt 4, f(%d, %d, t) lt 4, \\' \
            % (p0[0], p1[0], p0[1], p1[1])

for i in range(0, len(wp) - 1):
    p0 = wp[i]
    p1 = wp[i + 1]
    for axis in [0, 1]:
        P0 = p0[axis]
        P5 = p1[axis]
        s_d1 = d1s[i][axis]
        e_d1 = d1s[i + 1][axis]
        s_d2 = d2s[axis][i]
        e_d2 = d2s[axis][i + 1]
        P1 = 1.0 / 5.0 * s_d1 + P0
        P2 = 1.0 / 20.0 * s_d2 + 2 * P1 - P0
        P4 = P5 - 1.0 / 5.0 * e_d1
        P3 = 1.0 / 20.0 * e_d2 + 2 * P4 - P5
        print ' ' * axis,
        print 'S5(' + str(P0) + ', ' \
                + str(P1) + ', ' \
                + str(P2) + ', ' \
                + str(P3) + ', ' \
                + str(P4) + ', ' \
                + str(P5) + ', t) lt -1' \
                + ', \\'

for i in range(0, len(wp) - 1):
    p0 = wp[i]
    p1 = wp[i + 1]
    for axis in [0, 1]:
        P0 = p0[axis]
        P3 = p1[axis]
        P1 = P0 + (1.0 / 3.0) * d1s[i][axis]
        P2 = P3 - (1.0 / 3.0) * d1s[i + 1][axis]
        print ' ' * axis,
        print 'S3(' + str(P0) + ', ' \
                + str(P1) + ', ' \
                + str(P2) + ', ' \
                + str(P3) + ', t) lt 1' \
                + (', \\' if axis == 0 or i < len(wp) - 2 else '')

print ''

