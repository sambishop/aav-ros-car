
X = 0
Y = 1

class QuinticSegment:
    def __init__(self):
        self.P0 = None
        self.P1 = None
        self.P2 = None
        self.P3 = None
        self.P4 = None
        self.P5 = None

    def __str__(self):
        return '[P0:{0}, P1:{1}, P2:{2}, P3:{3}, P4:{4}, P5:{5}]'.\
                format(str(self.P0), str(self.P1), str(self.P2),
                        str(self.P3), str(self.P4), str(self.P5))

class CubicSegment:
    def __init__(self):
        self.P0 = None
        self.P1 = None
        self.P2 = None
        self.P3 = None

    def __str__(self):
        return '[P0:{0}, P1:{1}, P2:{2}, P3:{3}]'.\
                format(str(self.P0), str(self.P1),
                        str(self.P2), str(self.P3))

def quintic_spline_from_waypoints(waypoints):
    spline = []
    for wp in waypoints[:-1]:
        pair = []
        for a in [X, Y]:
            s = QuinticSegment()
            s.P0 = wp.axis[a].loc
            s.P5 = wp.next.axis[a].loc
            s.P1 = .2 * wp.axis[a].d1 + s.P0
            s.P2 = .05 * wp.axis[a].d2 + 2 * s.P1 - s.P0
            s.P4 = s.P5 - .2 * wp.next.axis[a].d1
            s.P3 = .05 * wp.next.axis[a].d2 + 2 * s.P4 - s.P5
            pair.append(s)
        spline.append((pair[0], pair[1]))
    return spline

def cubic_spline_from_waypoints(waypoints):
    spline = []
    for wp in waypoints[:-1]:
        pair = []
        for a in [X, Y]:
            s = CubicSegment()
            s.P0 = wp.axis[a].loc
            s.P3 = wp.next.axis[a].loc
            s.P1 = s.P0 + (1 / 3.0) * wp.axis[a].d1
            s.P2 = s.P3 - (1 / 3.0) * wp.next.axis[a].d1
            pair.append(s)
        spline.append((pair[0], pair[1]))
    return spline

