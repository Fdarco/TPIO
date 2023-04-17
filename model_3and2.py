from ctypes import alignment
import math
from time import altzone
from tokenize import endpats
from tracemalloc import start
import numpy as np


Lanewidth = 3.75
# the order is: West, South, East, North
# warning: the opposite exit can't be wider than the approach
approachLanes = np.array([3, 2, 3, 2])
approachWidths = approachLanes * Lanewidth
exitLanes = np.array([3, 2, 3, 2])
exitWidth = exitLanes * Lanewidth

# 每一个 connection 中，第一个参数是起点车道编号 
# 第二个参数是终点车道编号
# 车道编号都是内侧车道大于外侧车道
# the index method is showed as below.
#        9  8| 8  9
#       __ __|__ __
#            |
#  2 |       |       | 7
#  1 |       |       | 6
#  0 |_______|_______| 5
#  0 |       |       | 5
#  1 |       |       | 6
#  2 |       |       | 7
#       __ __|__ __
#       4  3 | 3  4
# 第三个参数表示从哪个方向转向哪个方向
# 第四个参数用于将栅格转化为对称的，用于 hybridAstar.py 的 getMapIndex() 函数
connections = [
    (0, 8, 'wn', 0, 'l'), 
    (1, 6, 'we', 0, 's'), 
    (2, 7, 'we', 0, 's'),
    (2, 4, 'ws', 0, 'r'),
    (3, 0, 'sw', 1, 'l'),
    (3, 8, 'sn', 1, 's'),
    (4, 9, 'sn', 1, 's'),
    (4, 7, 'se', 1, 'r'),
    (5, 3, 'es', 2, 'l'),
    (6, 1, 'ew', 2, 's'),
    (7, 2, 'ew', 2, 's'),
    (7, 9, 'en', 2, 'r'),
    (8, 5, 'ne', 3, 'l'),
    (8, 3, 'ns', 3, 's'),
    (9, 4, 'ns', 3, 's'),
    (9, 2, 'nw', 3, 'r')
]


v = 12.5
steer_max = 0.63
Lw = 4  # see Fig. 1 in paper <trajectory planning in 
Lf = 0  # full spatiotemporal domain at intersection
Lr = 0  # with mixed flows>
Lb = 2  # 
Lw_zone = 5
Lb_zone = 3
R = math.sqrt(Lw_zone * Lw_zone + Lb_zone * Lb_zone) / 2
r_min = Lw / math.sin(steer_max)
dt = 0.05

MapBound = max(max(approachWidths), max(exitWidth)) + 2*Lw   # 栅格化边界
Bound = max(max(approachWidths), max(exitWidth)) + 3*Lw   # Fig. 5 阴影部分边界

GridSize = v * dt / (np.sqrt(2) - 0.01)   # 空间栅格化粒度
ThetaSize = math.radians(1)   # 角度栅格化粒度


SafeRadius = 5
Safe_Half_Theta = math.radians(30) / 2


GreenTime = 10 / dt
YellowTime = 2 / dt
PhaseTime = GreenTime + YellowTime
CircleTime = 4 * PhaseTime


start_time = PhaseTime


minTime_HV = [51, 57, 24]
minTime_AV = [48, 58, 24]


# get the start vector of each lane of the approach
def getStartBase():
    StartPointBase = []
    StartAngleBase = []
    for i in range(approachLanes[0]):
        StartPointBase.append(
            (
                -max(approachWidths[3], exitWidth[1]) - 2*Lw, 
                -1/2*Lanewidth - Lanewidth*i
            )
        )
        StartAngleBase.append(0)

    for j in range(approachLanes[1]):
        StartPointBase.append(
            (
                1/2*Lanewidth + Lanewidth*j, 
                -max(approachWidths[0], exitLanes[2]) - 2*Lw
            )
        )
        StartAngleBase.append(math.pi/2)

    for k in range(approachLanes[2]):
        StartPointBase.append(
            (
                max(approachWidths[1], exitWidth[3]) + 2*Lw, 
                1/2*Lanewidth + Lanewidth*k
            )
        )
        StartAngleBase.append(math.pi)

    for l in range(approachLanes[3]):
        StartPointBase.append(
            (
                -1/2*Lanewidth - Lanewidth*l, 
                max(approachWidths[2], exitWidth[0]) + 2*Lw
            )
        )
        StartAngleBase.append(3*math.pi/2)

    return StartPointBase, StartAngleBase


StartPointBase, StartAngleBase = getStartBase()

# get end vector of each lane of the exit
def getEndBase():
    EndPointBase = []
    EndAngleBase = []
    for i in range(exitLanes[0]):
        EndPointBase.append(
            (
                -max(exitWidth[1], approachWidths[3]) - 2*Lw, 
                1/2*Lanewidth + Lanewidth*i
            )
        )
        EndAngleBase.append(math.pi)

    for j in range(exitLanes[1]):
        EndPointBase.append(
            (
                -1/2*Lanewidth - Lanewidth*j, 
                -max(exitWidth[2], approachWidths[0]) - 2*Lw
            )
        )
        EndAngleBase.append(3*math.pi/2)

    for k in range(exitLanes[2]):
        EndPointBase.append(
            (
                max(exitWidth[3], approachLanes[1]) + 2*Lw, 
                -1/2*Lanewidth - Lanewidth*k
            )
        )
        EndAngleBase.append(0)

    for l in range(exitLanes[3]):
        EndPointBase.append(
            (
                1/2*Lanewidth + Lanewidth*l, 
                max(exitWidth[0], approachWidths[2]) + 2*Lw
            )
        )
        EndAngleBase.append(math.pi/2)

    return EndPointBase, EndAngleBase


EndPointBase, EndAngleBase = getEndBase()


def getConnectionSettings():
    StartPoint = []
    EndPoint = []
    StartAng = []
    EndAng = []
    for i in range(len(connections)):
        StartPoint.append(StartPointBase[connections[i][0]])
        StartAng.append(StartAngleBase[connections[i][0]])
        EndPoint.append(EndPointBase[connections[i][1]])
        EndAng.append(EndAngleBase[connections[i][1]])

    return StartPoint, EndPoint, StartAng, EndAng


StartPoint, EndPoint, StartAng, EndAng = getConnectionSettings()


delta_G = v * dt
steer = {-1: -steer_max / 6, 
        -2: -steer_max / 3, 
        -3: -steer_max / 2, 
        -4: -2 * steer_max / 3, 
        -5: -5 * steer_max / 6, 
        -6: -steer_max, 0: 0, 
        1: steer_max / 6, 
        2: steer_max / 3, 
        3: steer_max / 2, 
        4: 2 * steer_max / 3,
        5: 5 * steer_max / 6, 
        6: steer_max
        }
r = {
    1: Lw / math.sin(steer[1]), 
    2: Lw / math.sin(steer[2]), 
    3: Lw / math.sin(steer[3]), 
    4: Lw / math.sin(steer[4]),
    5: Lw / math.sin(steer[5]), 
    6: Lw / math.sin(steer[6])
    }
ang = {
    1: 2 * math.asin((delta_G / 2) / r[1]), 
    2: 2 * math.asin((delta_G / 2) / r[2]),
    3: 2 * math.asin((delta_G / 2) / r[3]), 
    4: 2 * math.asin((delta_G / 2) / r[4]),
    5: 2 * math.asin((delta_G / 2) / r[5]), 
    6: 2 * math.asin((delta_G / 2) / r[6])
    }
G = {
    0: delta_G, 
    1: r[1] * ang[1], 
    2: r[2] * ang[2], 
    3: r[3] * ang[3], 
    4: r[4] * ang[4], 
    5: r[5] * ang[5],
    6: r[6] * ang[6]
    }


# ROUND: (x,y,radius)
Round = [
    [
        - approachWidths[3] - Lanewidth, 
        exitWidth[0] + Lanewidth, 
    ],
    [
        - exitWidth[1] - Lanewidth,
        - approachWidths[0] - Lanewidth,
    ],
    [
        approachWidths[1] + Lanewidth,
        - exitWidth[2] - Lanewidth,
    ],
    [
        exitWidth[3] + Lanewidth,
        approachWidths[2] + Lanewidth,
    ],
]


w = 800
h = 600

# 绘制弧线的半径
Lane_S = 5
# 用于在 pygame 中绘制道路边界线
# 共四组，分别是西、东、北、南四个方向
# 每组三条线，分别是左、中、右三条线
LineParam = [
    [
        (0, h / 2 - exitWidth[0] * 10), 
        (w / 2 - (approachWidths[3] + Lane_S) * 10, h / 2 - exitWidth[0] * 10)
        ],
    [
        (0, h / 2), 
        (w / 2 - (min(exitWidth[1], approachWidths[3]) + Lane_S) * 10, h / 2)
        ],
    [
        (0, h / 2 + approachWidths[0] * 10), 
        (w / 2 - (exitWidth[1] + Lane_S) * 10, h / 2 + approachWidths[0] * 10)
        ],

    [
        (w / 2 + (exitWidth[3] + Lane_S) * 10, h / 2 - approachWidths[2] * 10), 
        (w, h / 2 - approachWidths[2] * 10)
        ],
    [
        (w / 2 + (min(exitWidth[3], approachWidths[1])+ Lane_S) * 10, h / 2), 
        (w, h / 2)
        ],
    [
        (w / 2 + (approachWidths[1] + Lane_S) * 10, h / 2 + exitWidth[2] * 10), 
        (w, h / 2 + exitWidth[2] * 10)
        ],

    [
        (w / 2 - approachWidths[3] * 10, 0), 
        (w / 2 - approachWidths[3] * 10, h / 2 - (exitWidth[0] + Lane_S) * 10 + 3)
        ],
    [
        (w / 2, 0), 
        (w / 2, h / 2 - (min(approachWidths[2], exitWidth[0]) + Lane_S) * 10)
        ],
    [
        (w / 2 + exitWidth[3] * 10, 0), 
        (w / 2 + exitWidth[3] * 10, h / 2 - (approachWidths[2] + Lane_S) * 10 + 3)
        ],

    [
        (w / 2 - exitWidth[1] * 10, h), 
        (w / 2 - exitWidth[1] * 10, h / 2 + (approachWidths[0] + Lane_S) * 10)
        ],
    [
        (w / 2, h), 
        (w / 2, h / 2 + (min(approachWidths[0], exitWidth[2]) + Lane_S) * 10)
        ],
    [
        (w / 2 + approachWidths[1] * 10, h), 
        (w / 2 + approachWidths[1] * 10, h / 2 + (exitWidth[2] + Lane_S) * 10)
        ],
]

# 用于在 pygame 中绘制弧线
# 总共四个圆心，分别是左上，左下，右下，右上
RoundParam = [
    [
        (
            w / 2 - approachWidths[3] * 10 - Lane_S * 20 + 1, 
            h / 2 - exitWidth[0] * 10 - Lane_S * 20 + 1, 
            Lane_S * 20, 
            Lane_S * 20
            ),
        math.radians(270), 
        math.radians(0)
        ],
    [
        (
            w / 2 - exitWidth[1] * 10 - Lane_S * 20 + 1, 
            h / 2 + approachWidths[0] * 10, 
            Lane_S * 20, 
            Lane_S * 20
            ), 
        math.radians(0), 
        math.radians(90)
        ],
    [
        (
            w / 2 + approachWidths[1] * 10, 
            h / 2 + exitWidth[2] * 10, 
            Lane_S * 20, 
            Lane_S * 20
            ), 
        math.radians(90), 
        math.radians(180)
        ],
    [
        (
            w / 2 + exitWidth[3] * 10, 
            h / 2 - approachWidths[2] * 10 - Lane_S * 20 + 1, 
            Lane_S * 20, 
            Lane_S * 20
            ), 
        math.radians(180),
        math.radians(270)
        ],
]

# 在 pygame 中绘制进口道停车线
# 停车线有两个单位的宽度
# 分别是 西，南，东，北 四个方向的停车线
SingleParam = [
    [
        (w / 2 - (exitWidth[1] + Lane_S) * 10 - 2, h / 2), 
        (w / 2 - (exitWidth[1] + Lane_S) * 10, h / 2),
        (w / 2 - (exitWidth[1] + Lane_S) * 10, h / 2 + approachWidths[0] * 10), 
        (w / 2 - (exitWidth[1] + Lane_S) * 10 - 2, h / 2 + approachWidths[0] * 10)
        ],
    [
        (w / 2 + approachWidths[1] * 10, h / 2 + (exitWidth[2] + Lane_S) * 10), 
        (w / 2 + approachWidths[1] * 10, h / 2 + (exitWidth[2] + Lane_S) * 10 + 2),
        (w / 2, h / 2 + (exitWidth[2] + Lane_S) * 10 + 2), 
        (w / 2, h / 2 + (exitWidth[2] + Lane_S) * 10)],
    [
        (w / 2 + (exitWidth[3] + Lane_S) * 10, h / 2 - approachWidths[2] * 10), 
        (w / 2 + (exitWidth[3] + Lane_S) * 10 + 2, h / 2 - approachWidths[2] * 10),
        (w / 2 + (exitWidth[3] + Lane_S) * 10 + 2, h / 2), 
        (w / 2 + (exitWidth[3] + Lane_S) * 10, h / 2)
        ],
    [
        (w / 2 - approachWidths[3] * 10, h / 2 - (exitWidth[0] + Lane_S) * 10 - 2), 
        (w / 2, h / 2 - (exitWidth[0] + Lane_S) * 10 - 2),
        (w / 2, h / 2 - (exitWidth[0] + Lane_S) * 10), 
        (w / 2 - approachWidths[3] * 10, h / 2 - (exitWidth[0] + Lane_S) * 10)
        ],
]



def Point2Pol(x, y, ang):
    p1 = (
        x - Lb_zone / 2 * math.sin(ang) - Lr * math.cos(ang), 
        y + Lb_zone / 2 * math.cos(ang) - Lr * math.sin(ang)
        )
    p2 = (
        x + Lb_zone / 2 * math.sin(ang) - Lr * math.cos(ang), 
        y - Lb_zone / 2 * math.cos(ang) - Lr * math.sin(ang)
        )
    p3 = (
        x + Lb_zone / 2 * math.sin(ang) + Lw_zone * math.cos(ang),
        y - Lb_zone / 2 * math.cos(ang) + Lw_zone * math.sin(ang)
        )
    p4 = (
        x - Lb_zone / 2 * math.sin(ang) + Lw_zone * math.cos(ang),
        y + Lb_zone / 2 * math.cos(ang) + Lw_zone * math.sin(ang)
        )

    pol = [p1, p2, p3, p4]

    return pol

# point switch to polygen
# p1: left top
# p2: left bottom
# p3: right bottom
# p4: right top
def Point2Pol1(x, y, ang):
    p1 = (x - Lb / 2 * math.sin(ang), y + Lb / 2 * math.cos(ang))
    p2 = (x + Lb / 2 * math.sin(ang), y - Lb / 2 * math.cos(ang))
    p3 = (x + Lb / 2 * math.sin(ang) + Lw * math.cos(ang), y - Lb / 2 * math.cos(ang) + Lw * math.sin(ang))
    p4 = (x - Lb / 2 * math.sin(ang) + Lw * math.cos(ang), y + Lb / 2 * math.cos(ang) + Lw * math.sin(ang))

    pol = [p1, p2, p3, p4]

    return pol

# for debug, can be removed later
def Para2Vec(param):
    x, y, theta = param
    x2 = x + math.cos(theta)
    y2 = y + math.sin(theta)

    return x, y, x2, y2




if __name__ == '__main__':

    # 检验生成的起点和终点是否正确
    # 检验生成的边界圆是否正确

    from matplotlib import pyplot as plt

    plt.axis('equal')

    for i in range(len(StartPointBase)):
        x, y = StartPointBase[i]
        theta = StartAngleBase[i]
        param = x, y, theta 
        x1, y1, x2, y2 = Para2Vec(param)
        plt.plot([x1, x2], [y1, y2], color='blue')


    for j in range(len(EndPointBase)):
        x, y = EndPointBase[j]
        theta = EndAngleBase[j]
        param = x, y, theta
        x1, y1, x2, y2 = Para2Vec(param)
        plt.plot([x1, x2], [y1, y2], color='red')


    for rp in Round:
        plt.scatter(rp[0], rp[1], s=Lanewidth*100, color='green')

    plt.show()


    print(StartPoint)
    print(StartAng)
    print(EndPoint)
    print(EndAng)