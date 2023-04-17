import math
import numpy as np
from model import *


def B(n,i,t):
    return math.factorial(n)/(math.factorial(i)*math.factorial(n-i)) * t**i * (1-t)**(n-i)


def Bezier(p,n,c,dir):
    res = [(StartPoint[dir][0], StartPoint[dir][1], StartAng[dir])]
    temp = 0
    x1, y1 = p[0][0], p[0][1]
    for t in np.arange(0, 1+c, c):
        x0, y0 = 0,0
        for i in range(n+1):
            x0 += B(n,i,t) * p[i][0]
            y0 += B(n,i,t) * p[i][1]

        dx = x0 - x1
        dy = y0 - y1
        x1 = x0
        y1 = y0
        temp += math.sqrt(dx*dx + dy*dy)

        if temp>=v*dt:
            theta = math.atan2(dy, dx)
            res.append((x0,y0,theta))
            temp -= v*dt
            dx1 = x0-EndPoint[dir][0]
            dy1 = y0-EndPoint[dir][1]
            if math.sqrt((dx1*dx1) + (dy1*dy1)) < 2:
                break

    return res


def Bezier1(p,n):
    res = []
    temp = 0

    for t in range(n+1):
        x0, y0 = 0,0
        for i in range(n+1):
            x0 += B(n,i,t/n) * p[i][0]
            y0 += B(n,i,t/n) * p[i][1]

        res.append((x0,y0,p[t][2]))
        temp -= v*dt

    return res


if __name__ == '__main__':
    from matplotlib import pyplot as plt


    plt.axis('equal')


    for i in range(len(connections)):
        p0 = np.array(StartPoint[i])
        p1 = np.array(EndPoint[i])

        V = p1 - p0
        Vlength = np.sqrt(np.sum(np.square(V)))

        n0 = np.array([math.cos(StartAng[i]), math.sin(StartAng[i])])
        n1 = np.array([math.cos(EndAng[i]), math.sin(EndAng[i])])

        p2 = p0 + np.sum(np.multiply(n0, V))/2 * n0
        p3 = p1 - np.sum(np.multiply(n1, V))/2 * n1

        cp = [p0, p2, p3, p1]

        path = Bezier(cp, len(cp)-1, 0.0001, i)


        xx = [p[0] for p in path]
        yy = [p[1] for p in path]

        plt.plot(xx, yy, color='blue')
        plt.title(connections[i][2])

        plt.show()