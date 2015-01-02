import re

X = 0
Y = 1

class Axis:
    def __init__(self):
        self.loc = None
        self.d1 = None
        self.d2 = None

    def __str__(self):
        return '[loc:{0} d1:{1} d2:{2}]'.\
                format(str(self.loc), str(self.d1), str(self.d2))

class Waypoint:
    def __init__(self):
        self.axis = [Axis(), Axis()]
        self.prev = None
        self.next = None

    def __str__(self):
        return '[{0}, {1}]'.format(self.axis[X], self.axis[Y])

def dump_waypoints(waypoints):
    s = '#x,d1,d2,y,d1,d2\n'
    for wp in waypoints:
        s += ','.join([str(n) for n in [
            wp.axis[0].loc, wp.axis[0].d1, wp.axis[0].d2,
            wp.axis[1].loc, wp.axis[1].d1, wp.axis[1].d2]])
        s += '\n'
    return s

def parse_waypoints(s):
    waypoints = []
    for line in s.split('\n'):
        if line.startswith('#') or line.strip() == '':
            continue
        tokens = [ float(token.strip()) for token in line.split(',') ]
        if len(tokens) != 6:
            raise ValueError('incorrect number of values')
        wp = Waypoint()
        wp.axis[X].loc = tokens[0]
        wp.axis[X].d1 = tokens[1]
        wp.axis[X].d2 = tokens[2]
        wp.axis[Y].loc = tokens[3]
        wp.axis[Y].d1 = tokens[4]
        wp.axis[Y].d2 = tokens[5]
        waypoints.append(wp)
    return waypoints

