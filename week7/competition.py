#! /usr/bin/python3

from brickpi3 import BrickPi3

import numpy as np
from helper import RobotBase, Canvas, Map
from time import sleep
from math import pi, radians, degrees

import matplotlib.pyplot as plt

canvas = Canvas(210)
mymap = Map()

# draw wall
# Definitions of walls
# a: O to A
# b: A to B
# c: C to D
# d: D to E
# e: E to F
# f: F to G
# g: G to H
# h: H to O
mymap.add_wall((0,0,0,168));        # a
mymap.add_wall((0,168,84,168));     # b
mymap.add_wall((84,126,84,210));    # c
mymap.add_wall((84,210,168,210));   # d
mymap.add_wall((168,210,168,84));   # e
mymap.add_wall((168,84,210,84));    # f
mymap.add_wall((210,84,210,0));     # g
mymap.add_wall((210,0,0,0));        # h
canvas.drawLines(mymap.walls)
name_list = ["a","b","c","d","e","f","g","h"]


robot = RobotBase(BrickPi3.PORT_B, BrickPi3.PORT_C,BrickPi3.PORT_D, BrickPi3.PORT_4, mymap, p_start=(84.0,30.0,0), debug_canvas=canvas)

canvas.drawParticles(robot.p_tuples, robot.p_weights)

cfg_scan_a_range = (radians(-180), radians(180), radians(2))


def calibrate_and_getbottle():
    obstacles = robot.get_obstacles(-pi, pi, radians(2), DEBUG=True)

    walls_likelyhood = []
    _, walls = robot.identify_bottle(obstacles, walls_likely=walls_likelyhood)
    robot.update_pos(walls, walls_likelyhood)

    bottles_likelyhood = []
    bottles, _ = robot.identify_bottle(obstacles, bottles_likely=bottles_likelyhood)

    if bottles:
        max_likely = None
        max_rad = None
        for (_rad,_dis),(_b_like, _w_like) in zip(bottles, bottles_likelyhood):
            if _b_like/_w_like < 100:
                continue

            if max_likely is None or max_likely > _b_like:
                max_likely = _b_like
                max_rad = _rad

        if max_likely and max_likely>1.0e-100:
            print(f"go to bottle with likely {max_likely}")
            robot.to_relative_turn(max_rad)
            robot.touch_bottle(300)

        return True
    else:
        return False




def areaA():

    if robot.to_waypoint(126, 42):
        return True

    if calibrate_and_getbottle():
        return True

    if robot.to_waypoint(168, 42):
        return True

    if calibrate_and_getbottle():
        return True




def areaB():

    if robot.to_waypoint(126, 110):
        # hit bottle while travel to B, abort early
        return True

    if calibrate_and_getbottle():
        return True

    if robot.to_waypoint(126, 168):
        return True

    if calibrate_and_getbottle():
        return True

    return False


def areaC():
    if robot.to_waypoint(42, 60):
        return True

    if  calibrate_and_getbottle():
        return True

    if robot.to_waypoint(42, 106):
        return True

    if calibrate_and_getbottle():
        return True

    return False

#####################
#       Main      #
#####################

b_done =areaB()

a_done = areaA()

c_done = areaC()

robot.to_waypoint(60,30)
c_calibrate_obs = robot.get_obstacles(-pi, pi, radians(2), DEBUG=True)

if not c_done:
    areaC()

if not a_done:
    areaA()

if not b_done:
    areaB()

walls_likelyhood = []
_, walls = robot.identify_bottle(c_calibrate_obs,walls_likely=walls_likelyhood)
robot.update_pos(walls, walls_likelyhood)
robot.to_waypoint(84,30)
