from sqlite3 import Time
import sys
from tkinter import Spinbox
from point import Point
from limit import IsLimitPoint
from dubins import dubins, mod2pi
from conflict import IsConflictPoint
from model import *

def SelectMinPoint(set):
    index = 0
    selected_index = -1
    min_cost = sys.maxsize
    for p in set:
        if p.cost < min_cost:
            min_cost = p.cost
            selected_index = index
        index += 1
    return selected_index


def IsStartPoint(start, p):
    if start == p:
        return True
    return False


def IsEndPoint(dir, p):
    dx = abs(EndPoint[dir][0]-p.x)
    dy = abs(EndPoint[dir][1]-p.y)
    if math.sqrt((dx*dx) + (dy*dy)) < 2:
        return True
    return False


def BuildPath(start,p):
    path = []

    while True:
        path.append((p.x, p.y, p.theta))
        if IsStartPoint(start, p):
            break
        else:
            p = p.parent

    path.reverse()

    return path


def GetMapIndex(param, dir):
    i = connections[dir][3]
    
    map_size = int(2 * MapBound / GridSize + 1)
    if i == 0:
        x, y, theta = param
    elif i == 1:
        x, y, theta = param[1], -param[0], mod2pi(param[2]-math.radians(90))
    elif i == 2:
        x, y, theta = -param[0], -param[1], mod2pi(param[2]-math.radians(180))
    elif i == 3:
        x, y, theta = -param[1], param[0], mod2pi(param[2]-math.radians(270))

    index1 = int(min(int((x+MapBound)/GridSize), map_size-1))
    index2 = int(min(int((y+MapBound)/GridSize), map_size-1))
    index3 = int(min(int(theta/ThetaSize), int(math.radians(360)/ThetaSize+1)-1))

    return index1, index2, index3


def HybirdAstar(dir, t, PathInfo, TimeInfo, TypeInfo):
    map_size = int(2*MapBound/GridSize+1)
    MapSet = np.empty((map_size,map_size,int(math.radians(360)/ThetaSize+1)), dtype=bool)
    MapSet.fill(False)
    OpenSet = []


    param = StartPoint[dir][0], StartPoint[dir][1], StartAng[dir]
    StartP = Point(param)
    StartP.t = t
    StartP.cost = 0

    OpenSet.append(StartP)

    count = 0

    while True:
        # print(OpenSet)
        count += 1

        index = SelectMinPoint(OpenSet)

        if(len(OpenSet)==0):
            # print('set = 0')
            # logging.warning("set=0")
            return -1

        p = OpenSet.pop(index)

        # print(p)

        # if count % 50 == 0:
        #     print(p)

        if(IsEndPoint(dir, p)):
            path = BuildPath(StartP, p)
            return path

        IndexSet = []
        # [-1,0,1]->[左转，直行，右转]
        for i in [-1, 0, 1]:
            # print(i)
            param, next_turn = p.point_iter(i)

            index_m1_, index_m2_, index_m3_ = GetMapIndex(param, dir)
            # if p.t+1 - t > 1.25 * minTime_AV[int(dir % 3)]:
            #     continue
            if MapSet[index_m1_][index_m2_][index_m3_] == True:
                # print('This point has been searched')
                continue
            if IsConflictPoint(param, (p.t+1), PathInfo, TimeInfo, TypeInfo):
                # print("ConflictErr")
                continue
            if not IsLimitPoint(dir, param):
                # print("LimitErr")
                continue
            IndexSet.append([index_m1_, index_m2_, index_m3_])
            p_temp = Point(param)
            p_temp.parent = p
            p_temp.t = p_temp.parent.t+1
            p_temp.turn = next_turn
            p_temp.get_G()
            p_temp.get_H(dir)
            # print('H =', p_temp.H, param)
            p_temp.get_cost()
            OpenSet.append(p_temp)

        for item in IndexSet:
            MapSet[item[0]][item[1]][item[2]] = True

if __name__ == '__main__':
    from matplotlib import pyplot as plt

    qr = 100 # car flow (veh/h)
    T = 30  # amount of simulation time
    s = 0  # random seed
    mtd = 1  # minimum time headway
    ratio = 1  # ratio of AVs

    from generate_flow import generateCarFlow

    TimeInfo, CarInfo, TypeInfo = generateCarFlow(qr, T, s, mtd, ratio)
    print('TimeInfo', TimeInfo)

    DirecInfo = [connections[i][2] for i in CarInfo]
    print('DirecInfo: ', DirecInfo)

    SPInfo = [StartPoint[j] for j in CarInfo]
    print('SPInfo: ', SPInfo)

    ENInfo = [EndPoint[k] for k in CarInfo]
    print('ENInfo: ', ENInfo)

    total = len(CarInfo)
    PathInfo = [[] for i in range(total)]

    cnt = 0
    while cnt < total:

        print('='*30)

        path = HybirdAstar(
            CarInfo[cnt], 
            TimeInfo[cnt], 
            PathInfo, 
            TimeInfo, 
            TypeInfo
            )
        
        print(path)

        if path == -1:
            TimeInfo[cnt] += 1
            continue

        PathInfo[cnt] = path

        cnt += 1

    plt.axis('equal')
    # fig = plt.figure(figsize=(5, 5))
    # ax = fig.add_subplot(111)

    # ax.spines['top'].set_color('none')
    # ax.spines['right'].set_color('none')
    # ax.xaxis.set_ticks_position('bottom')
    # ax.spines['bottom'].set_position(('data', 0))
    # ax.yaxis.set_ticks_position('left')
    # ax.spines['left'].set_position(('data', 0))

    for path in PathInfo:
        xx = [point[0] for point in path]
        yy = [point[1] for point in path]

        plt.plot(xx, yy)

    plt.show()