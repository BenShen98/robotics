#! /usr/bin/python3

from brickpi3 import BrickPi3

from helper import RobotBase, Canvas, Map
from time import sleep
from math import pi

robot = RobotBase(BrickPi3.PORT_B, BrickPi3.PORT_C)
canvas = Canvas(40)
mymap = Map()


# draw wall
wall_corner = [(0,0), (0,40), (40,40), (40,0)]
for i in range(3):
    mymap.add_wall((*wall_corner[i], *wall_corner[i+1]))
mymap.add_wall((*wall_corner[3], *wall_corner[0]))

# draw lines
canvas.drawLines(mymap.walls)

# init
for _ in range(4):
    for _ in range(4):
        robot.to_relative_forward(10) # *task unpack to arguments
        canvas.drawParticles(robot.p_tuples, robot.p_weights)
        print (f"est postion:{robot.get_est_pos()}")
        sleep(1)

    robot.to_relative_turn(0.5*pi)
    canvas.drawParticles(robot.p_tuples, robot.p_weights)
    sleep(1)