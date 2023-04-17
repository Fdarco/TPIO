from model import *


def GetCross(p, p1, p2):
    return (p2[0] - p1[0]) * (p[1] - p1[1]) - (p[0] - p1[0]) * (p2[1] - p1[1])

def IsPointInRec(p, rec):
    p1,p2,p3,p4 = rec
    return GetCross(p,p1,p2) * GetCross(p,p3,p4) >= 0 and GetCross(p,p2,p3) * GetCross(p,p4,p1) >= 0


def circleConfict(x1, y1, x2, y2):
    dx = x1 - x2
    dy = y1 - y2
    
    return pow(dx, 2) + pow(dy, 2) < pow(Lanewidth, 2)

# 判断节点是否在道路的外面
# 弧线的部分也需要判断
def isConflictRec(param):
    x, y, theta = param
    pol = Point2Pol1(x, y, theta)

    for p in pol:
        if p[0] < -approachWidths[3] and p[1] > exitWidth[0] + Lanewidth:
            return True 

        if p[0] < -approachWidths[3] - Lanewidth and p[1] > exitWidth[0]:
            return True 

        if p[0] < -exitWidth[1] - Lanewidth and p[1] < -approachWidths[0]:
            return True 

        if p[0] < -exitWidth[1] and p[1] < -approachWidths[0] - Lanewidth:
            return True 

        if p[0] > approachWidths[1] and p[1] < -exitWidth[2] - Lanewidth:
            return True

        if p[0] > approachWidths[1] + Lanewidth and p[1] < -exitWidth[2]:
            return True

        if p[0] > exitWidth[3] + Lanewidth and p[1] > approachWidths[2]:
            return True

        if p[0] > exitWidth[3] and p[1] > approachWidths[2] + Lanewidth:
            return True

        for item in Round:
            if circleConfict(p[0], p[1], item[0], item[1]):
                return True
        
    return False


def isConflictCar(param1, param2):
    x1, y1, theta1 = param1
    x2, y2, theta2 = param2
    x1c = x1 - Lw/2*math.cos(theta1)   # 质心
    y1c = y1 - Lw/2*math.sin(theta1)
    x2c = x2 - Lw/2*math.cos(theta2)
    y2c = y2 - Lw/2*math.sin(theta2)

    if pow(x1c-x2c, 2) + pow(y1c-y2c, 2) > pow(3*R, 2):
        return False

    Pol1 = Point2Pol(x1, y1, theta1)
    Pol2 = Point2Pol(x2, y2, theta2)
    for p in Pol1:
        if IsPointInRec(p, Pol2):
            return True
    for p in Pol2:
        if IsPointInRec(p, Pol1):
            return True

    return False


def isConflictCar1(param1, param2):
    x1, y1, theta1 = param1
    x2, y2, theta2 = param2
    x1c = x1 - Lw/2*math.cos(theta1)
    y1c = y1 - Lw/2*math.sin(theta1)
    x2c = x2 - Lw/2*math.cos(theta2)
    y2c = y2 - Lw/2*math.sin(theta2)

    if (x1c-x2c)*(x1c-x2c) + (y1c-y2c)*(y1c-y2c) <= 4*R*R:
        return True

    return False


def IsConflictPoint(param, t, PathInfo, TimeInfo, TypeInfo):
    if isConflictRec(param):
        # print('conflict with bound')
        return True

    for index0 in range(len(PathInfo)):
        if PathInfo[index0] == []:
            continue
        if t < TimeInfo[index0] or t - TimeInfo[index0] >= len(PathInfo[index0]):
            continue
        index1 = t - TimeInfo[index0]
        if TypeInfo[index0] == 1:
            if isConflictCar(param, PathInfo[index0][index1]):
                return True
        else:
            # pass
            # if isConflictCar(param, PathInfo[index0][index1]):
            #     return True
            i_min = int(0.9*index1)
            i_max = min(int(1.1*index1)+1, len(PathInfo[index0])-1)
            for index2 in np.arange(i_min, i_max, 1):
                if isConflictCar1(param, PathInfo[index0][index2]):
                    return True

    return False


def IsConflictPoint1(param, t, PathInfo, TimeInfo):
    for index0 in range(len(PathInfo)):
        if PathInfo[index0] == []:
            continue
        if t < TimeInfo[index0] or t - TimeInfo[index0] >= len(PathInfo[index0]):
            continue
        index1 = t - TimeInfo[index0]
        if isConflictCar(param, PathInfo[index0][index1]):
            return True

    return False


if __name__ == '__main__':
    # from matplotlib import pyplot as plt
    print(approachWidths)
    print(exitWidth)

    print('='*10, 'Start Point Conflict Test', '='*10)

    for i in range(len(StartPointBase)):
        print('*'*30)
        print((StartPointBase[i][0], StartPointBase[i][1], StartAngleBase[i]))

        if isConflictRec((StartPointBase[i][0], StartPointBase[i][1], StartAngleBase[i])):
            print('\033[31mConflict with Bound\033[0m')
        else:
            print('\033[34mSafe point\033[0m')


    print('='*10, 'End Point Conflict Test', '='*10)

    for i in range(len(EndPointBase)):
        print('*'*30)
        print((EndPointBase[i][0], EndPointBase[i][1], EndAngleBase[i]))

        if isConflictRec((EndPointBase[i][0], EndPointBase[i][1], EndAngleBase[i])):
            print('\033[31mConflict with Bound\033[0m')
        else:
            print('\033[34mSafe point\033[0m')