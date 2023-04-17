import sys
from model import *
from dubins import dubins, mod2pi

turn_cost = {0:0, 1:0}   # 0.005-0.01, 0.025

class Point:
    def __init__(self, param):
        self.x, self.y, self.theta = param
        self.turn = 0
        self.G = 0
        self.H = 0
        self.cost = sys.maxsize
        self.parent = None

    def point_iter(self, i):
        next_x = self.x + delta_G*math.cos(self.theta)
        next_y = self.y + delta_G*math.sin(self.theta)
        next_turn = max(min(self.turn+i, 6), -6)
        next_theta = mod2pi(self.theta + delta_G/Lw*math.tan(steer[next_turn]))

        param = next_x, next_y, next_theta
        return param, next_turn

    # calculate the distance traveled
    def get_G(self):
        self.G = self.parent.G + G[abs(self.turn)]

    # calculate the Dubins cost to the target
    def get_H(self, dir):
        start = self.x, self.y, self.theta
        self.H = dubins(start, dir)

    def get_cost(self):
        self.cost = self.G + self.H

    def __str__(self):
        return '({}, {}, {})'.format(self.x, self.y, self.theta)


if __name__ == "__main__":
    print(steer)
    print(r)
    print(ang)
    print(G)
