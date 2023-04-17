import math
from model import *

# 在limitAng内是错误的角度
limitAngBase = {
    'wn' : [(math.pi, 3*math.pi/2)],
    'we' : [(math.pi/2, 3*math.pi/2)],
    'ws' : [(math.pi/2, math.pi)],
    'sw' : [(3*math.pi/2, 2*math.pi)],
    'sn' : [(math.pi, 2*math.pi)],
    'se' : [(math.pi, 3*math.pi/2)],
    'es' : [(0, math.pi/2)],
    'ew' : [(3*math.pi/2, 2*math.pi), (0, math.pi/2)],
    'en' : [(3*math.pi/2, 2*math.pi)],
    'ne' : [(math.pi/2, math.pi)],
    'ns' : [(0, math.pi)],
    'nw' : [(0, math.pi/2)],
}


# 在limitArea内是正确的区域[x_min, x_max, y_min, y_max]
limitAreaBase = {
    'wn' : [
        -Bound, exitWidth[3], 
        -approachWidths[0], Bound
        ],
    'we' : [
        -Bound, Bound, 
        -approachWidths[0], 0
        ],
    'ws' : [
        -Bound, 0,
        -Bound, 0
        ],
    'sw' : [
        -Bound, approachWidths[1], 
        -Bound, exitWidth[3]
        ],
    'sn' : [
        0, approachWidths[1], 
        -Bound, Bound
        ],
    'se' : [
        0, Bound, 
        -Bound, 0
        ],
    'es' : [
        -exitWidth[1], Bound, 
        -Bound, approachWidths[2]
        ],
    'ew' : [
        -Bound, Bound, 
        0, approachWidths[2]
        ],
    'en' : [
        0, Bound, 
        0, Bound
        ],
    'ne' : [
        -approachWidths[3], Bound, 
        -exitWidth[2], Bound
        ],
    'ns' : [
        -approachWidths[3], 0, 
        -Bound, Bound
        ],
    'nw' : [
        -Bound, 0, 
        0, Bound
        ],
}


def getLimitSettings():
    limitAng = []
    limitArea = []
    for i in range(len(connections)):
        limitAng.append(limitAngBase[connections[i][2]])
        limitArea.append(limitAreaBase[connections[i][2]])

    return limitAng, limitArea


limitAng, limitArea = getLimitSettings()


def IsLimitPoint(dir, param):
    x, y, theta = param

    for item in limitAng[dir]:
        if theta>=item[0] and theta<=item[1]:
            return False

    Pol = Point2Pol(x, y, theta)
    for item in Pol:
        if item[0] < limitArea[dir][0] or item[0] > limitArea[dir][1] \
            or item[1] < limitArea[dir][2] or item[1] > limitArea[dir][3]:
            return False

    return True
