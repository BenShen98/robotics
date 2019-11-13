#! /usr/bin/python3

from brickpi3 import BrickPi3

from helper import RobotBase, Canvas, Map
from time import sleep
from math import pi
import numpy as np

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


# init
count = 200
pos = (180,42,0)
robot = RobotBase(BrickPi3.PORT_B, BrickPi3.PORT_C, BrickPi3.PORT_D, BrickPi3.PORT_1,mymap, p_start=pos, p_count=count, debug_canvas=canvas)
canvas.drawParticles(robot.p_tuples, robot.p_weights)
sleep(1)

# hijack points
print("particles start from (180,42,0), but robot is at (190,42,0)")

var = np.random.uniform(-20,20, (count,2))
hijack=[]
for v in var:
    hijack.append( (pos[0]+v[0], pos[1]+v[1], pos[2]) )
robot.p_tuples = hijack
canvas.drawParticles(robot.p_tuples, robot.p_weights)
sleep(1)

for _ in range(10):
    robot.sonar_calibrate(20)
    canvas.drawParticles(robot.p_tuples, robot.p_weights)
    print(f"est pos: {robot.get_est_pos()}")

print("correct location based on sensor reading is 190,42")


