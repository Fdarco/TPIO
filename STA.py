import numpy as np
from conflict import IsConflictPoint1
from bezier import Bezier
from model import *

STA_point_ = [[] for i in range(len(connections))]


# 获取每个方向的控制点，一共是4个控制点
# 分别是起点，终点，起点沿着起点方向前进全部的距离
# 终点沿着终点的反方向前进一半的距离
def GetSTAPoint():
    for i in range(len(connections)):
        p0 = np.array(StartPoint[i])   # 起点位置
        p1 = np.array(EndPoint[i])   # 终点位置
        V = p1 - p0   # 由起点指向终点的向量

        # 起点方向单位向量
        n0 = np.array([math.cos(StartAng[i]), math.sin(StartAng[i])])
        # 终点方向单位向量
        n1 = np.array([math.cos(EndAng[i]), math.sin(EndAng[i])])

        p2 = p0 + np.sum(np.multiply(n0, V)) * n0
        p3 = p1 - np.sum(np.multiply(n1, V)) / 2 * n1

        cp = [p0, p2, p3, p1]

        STA_point_[i] = Bezier(cp, len(cp)-1, 0.0001, i)

# @return lane(0-3), state([True,Flase]->[Green,Yellow])
def GetSignalState(dir, t):
    if dir % 4 == 3:
        return True

    lane = int(t / PhaseTime) % 4
    state = t % PhaseTime < GreenTime
    if int(dir / 4) == lane and state == True:
        return True

    return False


def STA(dir, t, PathInfo, TimeInfo):
    if not GetSignalState(dir, t):
        # print('signal conflict')
        return -1
    else:
        for index in range(len(STA_point_[dir])):
            if IsConflictPoint1(STA_point_[dir][index], t+index, PathInfo, TimeInfo):
                # print('point conflict')
                return -1

    return STA_point_[dir]


if __name__=="__main__":
    from matplotlib import pyplot as plt

    plt.axis('equal')

    GetSTAPoint()

    for i in range(len(STA_point_)):
        xx = [p[0] for p in STA_point_[i]]
        yy = [p[1] for p in STA_point_[i]]

        plt.plot(xx, yy)

    plt.show()

    # for Point in STA_point_:
    #     print(Point)
    #     x = []
    #     y = []
    #     for p in Point:
    #         x.append(p[0])
    #         y.append(p[1])
    #     plt.plot(x, y, color='b', linewidth=1, linestyle='--')
    #
    # plt.show()