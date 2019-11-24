#! /usr/bin/python3

import brickpi3
import numpy as np
from math import cos, sin, atan2, pi, sqrt, exp,radians,degrees
import time
from bisect import bisect_left
import logging
import matplotlib.pyplot as plt

"""
Direction defintion
1. Let postive x-axi point to right
2. Let postive y-axi point to up
3. Let angle theta be the angle arctan(y/x)

Unit of measurement
1. Unit of distance is cm
2. Unit of angle is radian
3. Prameter proportional to angle or distance need to be normalised
"""

# characteristics for sonar sensor
max_sonar_distance = 150    # max reliable measuring distance for sonar (in cm)
min_sonar_angle_cos = cos(30 *pi/180)    # cos of cutoff angle for sonar
sonar_var = 3**2    #sonar gaussian likelihood variance
sonar_K = 0.01    # sonar gaussian likelihood offset value

class RobotBase(brickpi3.BrickPi3):
    def __init__(self, M_LEFT, M_RIGHT, M_SONAR, S_SONAR, map, p_start=(0.0,0.0,0.0),*, p_count=200, gaussian_e=(0,0.03), gaussian_f=(0, 0.002/180*pi), gaussian_g=(0,0.5/180*pi), debug_canvas=None):
        # BP init
        super(RobotBase, self).__init__()
        self.M_LEFT = M_LEFT
        self.M_RIGHT = M_RIGHT
        self.M_SONAR = M_SONAR
        self.S_SONAR = S_SONAR

        self.S_TOUCH_RIGHT = self.PORT_1
        self.S_TOUCH_LEFT = self.PORT_2


        self.set_sensor_type(self.S_SONAR, self.SENSOR_TYPE.NXT_ULTRASONIC)
        self.set_sensor_type(self.S_TOUCH_RIGHT, self.SENSOR_TYPE.NXT_TOUCH)
        self.set_sensor_type(self.S_TOUCH_LEFT, self.SENSOR_TYPE.NXT_TOUCH)

        self.offset_motor_encoder(self.M_LEFT, self.get_motor_encoder(M_LEFT))
        self.offset_motor_encoder(self.M_RIGHT, self.get_motor_encoder(M_RIGHT))
        self.offset_motor_encoder(self.M_SONAR, self.get_motor_encoder(M_SONAR))

        # characteristics for robot turning
        self.stright_dps = 500
        #self.stright_dpcm = (7870/280 + 5949/210)/2 # degree of motor rotation to move 1 cm
        self.stright_dpcm = 28
        self.turn_dps = 100
        self.turn_dpradian = 4250.0/5/2/pi # degree of motor rotation to trun 1 radian

        #TODO: characteristics for turntable
        # self.sonar_epd = - 1260.0/180 # truntable (encoder) rotation per rotation, NOTE:the negative to make direction consistant
        self.sonar_epr = - 1260 / pi

        # localisation init
        self.map = map

        # NOTE: p_weights and p_tuples need have same order
        placement_error_t = 20/180*pi
        placement_error_xy = 0.1
        self.p_count = p_count
        t_errors = np.random.normal(0, placement_error_t**2,  self.p_count)
        xy_errors = np.random.normal(0, placement_error_xy**2,  size=(self.p_count,2))
        self.p_weights = [1.0/p_count] * p_count

        temp = []
        for (x,y), t in zip(xy_errors, t_errors):
            temp.append( (p_start[0]+x, p_start[1]+y, p_start[2]+t) )
        self.p_tuples = temp


        # config for gaussian noise, all normalised by cm, or by radian
        self.gaussian_e = gaussian_e
        self.gaussian_f = gaussian_f
        self.gaussian_g = gaussian_g

        # debug
        self.debug_canvas = debug_canvas


        #wait hardware to init
        time.sleep(1)

    def __del__(self):
        #wait hardware to init
        self.reset_all()
        time.sleep(1)

    def get_sonar_dis(self,relative_rad=None):

        # go to position and measure
        if relative_rad is not None:
            self.set_sonar_rad(relative_rad, blocking=True)

        while True:
            try:
                sonar_distance = self.get_sensor(self.S_SONAR)
                break
            except:
                logging.warning("invalid sonar data")
                time.sleep(0.5)

        return (self.get_sonar_rad(), sonar_distance)

    def get_sonar_rad(self):
        return self.get_motor_encoder(self.M_SONAR)/self.sonar_epr

    def set_sonar_rad(self, rad, blocking=False):
        encoder_target = int( rad * self.sonar_epr)
        self.set_motor_position(self.M_SONAR, encoder_target)

        if blocking:
            while abs(self.get_motor_encoder(self.M_SONAR) - encoder_target) > 2:
                pass
            self.set_motor_dps(self.M_SONAR, 0)

        return

    def get_nearest_obstacles(self,relative_start,relative_end,step,varthrs, DEBUG = False): # Return List ofBanana shape angle, parameters should be in radians
        """
        From sonar scan, find all banana shaped obstacles and return its center point and distance
        With respect to robot
        Return a list of (relative_rad, distance) of a obsticle
        """

        # Get reading
        read_dis = []
        read_rad = []
        for r in np.arange(relative_start, relative_end, step):
            rad, dis = self.get_sonar_dis(r)
            read_rad.append(rad)
            read_dis.append(dis)
        time.sleep(0.5)
        self.set_sonar_rad(0)
        # Start data processing
        result = []
        start = True
        # Find mid points of all banana shapes
        for i in range(2,len(read_rad)-3):
            if read_dis[i-2] == 255:
                continue
            _var = np.var(read_dis[i-2:i+3])
            if _var==0 and start:
                start_index = i
                start = False
            elif _var >= varthrs and not start:
                end_index= i-1
                start = True
                if (end_index - start_index)>10:
                    _rad = read_rad[int((start_index + end_index)/2)]
                    _dis = read_dis[int((start_index + end_index)/2)]
                    # min_idx = np.argmin(read_dis[start_index:end_index+1])
                    # _rad = read_rad[start_index+min_idx]
                    # _dis = min(read_dis[start_index:end_index+1])
                    result.append( (_rad, _dis) )
            else:
                pass

        if DEBUG:
            fig = plt.figure()
            ax = plt.subplot(111)
            ax.scatter(np.array(read_rad)*180/pi,read_dis,alpha=0.5, marker='x', c='blue', s=10)
            if result:
                rads, dises = np.array(result).transpose()
                ax.scatter(rads*180/pi,dises, marker='o',c='black', s=40)
            plt.grid(linewidth = 0.2)
            plt.savefig('scan.png')

        return result
      
    def get_pos_mean(self):
        tr = np.array(self.p_tuples).transpose()

        return (tr[0].mean(), tr[1].mean(), atan2( np.sin(tr[2]).mean(), np.cos(tr[2]).mean() ) )

    def get_pos_var(self):
        return np.var(self.p_tuples, axis=0)



    def sonar_calibrate(self, relative_rad=0.0):
        """
        Return True when sucessful, False otherwise
        """
        # TODO: use turntable

        # config
        max_invalid_rate = 0.9

        # get sonar_distance
        sonar_rad, sonar_distance = self.get_sonar_dis(relative_rad)


        # discard this reading if is outof the reliable range
        if sonar_distance>max_sonar_distance:
            logging.warning(f"Sonar get distance {sonar_distance}, exceed {max_sonar_distance} limit")
            return False

        # making cumulative weight table
        weight_table = [0] * self.p_count
        invalid_read = 0 # only count return None, not return 0

        debug_w = []

        for i, p in enumerate(self.p_tuples):
            _x, _y, _t = p
            _t += sonar_rad

            _w = self.p_weights[i]

            w = self.map.calculate_likelihood(_x, _y, _t, sonar_distance)

            if w is None:
                invalid_read += 1
                w = 0

            weight_table[i] = weight_table[i-1] + w*_w

            debug_w.append(w)

        # check if there are too many invalid read
        if invalid_read/self.p_count > max_invalid_rate:
            logging.warning(f"Too much invalid particles, get{invalid_read}")
            time.sleep(2)
            return False

        if invalid_read:
            logging.warning(f"Got {invalid_read} invalid particles")


        # generate next batch of particles
        temp = []
        rand = np.random.uniform(0, weight_table[-1], self.p_count)
        for r in rand:
            i = bisect_left(weight_table, r)
            temp.append(self.p_tuples[i])

        self.p_tuples = temp
        self.p_weights = [1/self.p_count] * self.p_count

        return True


    def to_waypoint(self, x, y, accuracy=3):
        # config
        max_forward = 20

        # get estimation of current location
        while True:
            est_x, est_y, est_t = self.get_pos_mean()
            logging.warning(f"at ({est_x},{est_y},{est_t}) to ({x},{y})")


            # calculate movement, abort it needed
            diff_x = x - est_x
            diff_y = y - est_y
            target_t = atan2(diff_y, diff_x)

            relative_t = target_t - est_t
            relative_t = normalise_anlge(relative_t) # bond turning angle to +- pi
            relative_d = sqrt(diff_x*diff_x + diff_y*diff_y)

            # break large increment to smaller
            if relative_d > max_forward:
                relative_d = max_forward

            logging.warning((relative_t, relative_d))
            # exit
            if relative_d < accuracy:
                logging.info(f"dis to waypoint: {relative_d}, require {accuracy}")
                return

            # perform movement and calibrate
            self.to_relative_turn(relative_t)
            self.sonar_calibrate()


            self.to_relative_forward(relative_d)

            self.sonar_calibrate()
            if self.debug_canvas:
                self.debug_canvas.drawParticles(self.p_tuples, self.p_weights, self.get_pos_mean())
                time.sleep(0.5)

            logging.warning(f"After calibration get var {self.get_pos_var()}")


            # TODO: use turntable to seek alternative measurement

    def touch_bottle(self, push_dps = 100):
        # reset encoder
        self.offset_motor_encoder(self.M_RIGHT, self.get_motor_encoder(self.M_RIGHT))
        self.offset_motor_encoder(self.M_LEFT, self.get_motor_encoder(self.M_LEFT))

        # move farward until touch sensor get touched
        self.set_motor_dps(self.M_RIGHT | self.M_LEFT, push_dps)
        while(  not (self.get_sensor(self.S_TOUCH_LEFT) or self.get_sensor(self.S_TOUCH_RIGHT)) ):
            pass
        self.set_motor_dps(self.M_RIGHT | self.M_LEFT, 0)

        # record how much it moved
        dis_moved_r = self.get_motor_encoder(self.M_RIGHT)
        dis_moved_l = self.get_motor_encoder(self.M_LEFT)
        max_dis = min(dis_moved_r, dis_moved_l)

        # move backward
        self.set_motor_dps(self.M_RIGHT | self.M_LEFT, - self.stright_dps)
        while (self.get_motor_encoder(self.M_LEFT)>=0 or self.get_motor_encoder(self.M_RIGHT)>=0):
            pass
        self.set_motor_dps(self.M_RIGHT | self.M_LEFT, 0)
        logging.info("moved r{dis_moved_r}, l{dis_moved_l}")

        # spread particles
        err_e = np.random.normal(*np.array(self.gaussian_e)*abs(max_dis*2), self.p_count)
        err_f = np.random.normal(*np.array(self.gaussian_f)*max_dis*2, self.p_count)
        for i in range(len(self.p_tuples)):
            _x, _y, _t = self.p_tuples[i]
            self.p_tuples[i] = (_x + cos(_t)*err_e[i],
                                _y + sin(_t)*err_e[i],
                                normalise_anlge( _t + err_f[i]))


    def to_relative_forward(self, distance):
        # update model
        inc_distance = np.random.normal(*np.array(self.gaussian_e)*abs(distance), self.p_count) + distance
        err_f = np.random.normal(*np.array(self.gaussian_f)*distance, self.p_count)
        for i in range(len(self.p_tuples)):
            _x, _y, _t = self.p_tuples[i]
            self.p_tuples[i] = (_x + cos(_t)*inc_distance[i],
                                _y + sin(_t)*inc_distance[i],
                                normalise_anlge( _t + err_f[i]))

        # perform movement
        self.offset_motor_encoder(self.M_RIGHT, self.get_motor_encoder(self.M_RIGHT))
        self.offset_motor_encoder(self.M_LEFT, self.get_motor_encoder(self.M_LEFT))
        self.set_motor_dps(self.M_RIGHT | self.M_LEFT, self.stright_dps )
        while (self.get_motor_encoder(self.M_RIGHT) < self.stright_dpcm*distance):
            pass
        self.set_motor_dps(self.M_RIGHT | self.M_LEFT, 0)



    def to_relative_turn(self, angle):
        # update model
        err_g = np.random.normal(*np.array(self.gaussian_g)*abs(angle), self.p_count)
        for i in range(len(self.p_tuples)):
            _x, _y, _t = self.p_tuples[i]
            self.p_tuples[i] = (_x ,
                                _y ,
                                normalise_anlge(_t + angle+ err_g[i]))

        # perform movement
        self.offset_motor_encoder(self.M_RIGHT, self.get_motor_encoder(self.M_RIGHT)) # reset encoder B
        self.offset_motor_encoder(self.M_LEFT, self.get_motor_encoder(self.M_LEFT)) # reset encoder C
        # Turn function
        if angle>0:
            self.set_motor_dps(self.M_LEFT, -self.turn_dps)
            self.set_motor_dps(self.M_RIGHT, self.turn_dps)
            while (self.get_motor_encoder(self.M_RIGHT) < angle*self.turn_dpradian):
                pass
        else:
            self.set_motor_dps(self.M_LEFT, self.turn_dps)
            self.set_motor_dps(self.M_RIGHT, -self.turn_dps)
            while (self.get_motor_encoder(self.M_RIGHT) > angle*self.turn_dpradian):
                pass
        self.set_motor_dps(self.M_RIGHT | self.M_LEFT, 0)

def normalise_anlge(theta):
    t = theta % (2*pi) #modulus will have the same sign as the denominator
    if t > pi:
        t -= 2*pi
    return t

def calDis(ax,ay,bx,by,x,y,theta):
    result = ((by-ay)*(ax-x) - (bx-ax)*(ay-y))/((by-ay)*cos(theta)-(bx-ax)*sin(theta))
    return result

def myswap(x,y):
    return y,x


# A Canvas class for drawing a map and particles:
#     - it takes care of a proper scaling and coordinate transformation between
#      the map frame of reference (in cm) and the display (in pixels)
class Canvas:
    def __init__(self,map_size=210):
        self.map_size    = map_size;    # in cm;
        self.canvas_size = 768;         # in pixels;
        self.margin      = 0.05*map_size;
        self.scale       = self.canvas_size/(map_size+2*self.margin);

        self.bearing_c = (240,190)
        self.bearing_r = 30

        _x, _y = self.bearing_c
        self.drawLine((_x-self.bearing_r,_y,_x+self.bearing_r,_y))
        self.drawLine((_x,_y-self.bearing_r,_x,_y+self.bearing_r))

    def drawLines(self, lines):
        for line in lines:
            self.drawLine(line)

    def drawPoints(self, points):
        for (x,y) in points:
            self.drawLine((x-1,y-1,x+1,y+1))
            self.drawLine((x-1,y+1,x+1,y-1))

    def drawLine(self,line):
        x1 = self.__screenX(line[0])
        y1 = self.__screenY(line[1])
        x2 = self.__screenX(line[2])
        y2 = self.__screenY(line[3])
        print ("drawLine:" + str((x1,y1,x2,y2)))

    def drawParticles(self,tuples, weights=None, show_est=None):
        plot_dp = 2
        if weights:
            display = [(self.__screenX(t[0]),self.__screenY(t[1])) + (round(t[2],plot_dp),) + (weights[i],)  for i, t in enumerate(tuples)]
        else:
            # without weights
            display = [(self.__screenX(t[0]),self.__screenY(t[1])) + (round(t[2],plot_dp),) for t in tuples]

        if show_est:
            _x, _y, _t = show_est
            x,y = self.bearing_c
            # dot for est center
            display.append( (self.__screenX(_x),self.__screenY(_y)) + (round(_t,plot_dp),) + (100,) )
            # dot for direction
            display.append( (self.__screenX(x + cos(_t)*self.bearing_r),self.__screenY(y + sin(_t)*self.bearing_r)) + (round(_t,plot_dp),) + (100,) )

        print ("drawParticles:" + str(display))

    def __screenX(self,x):
        return (x + self.margin)*self.scale

    def __screenY(self,y):
        return (self.map_size + self.margin - y)*self.scale

# A Map class containing walls
class Map:
    def __init__(self):
        self.walls = [];
        self.wall_list = ["a","b","c","d","e","f","g","h"]

    def add_wall(self,wall):
        self.walls.append(wall);

    def clear(self):
        self.walls = []

    def get_wall(self,wallname):
        idx = walllist.index(wallname)
        return self.walls[idx]
      
    def inside_map(self,x,y):
        if(x>=0 and x<=84 and y<=168 and y>=0): # section A on map
            return True
        elif(x>84 and x<=168 and y<=210 and y>=0): # section B on map
            return True
        elif(x>168 and x<=210 and y>=0 and y<= 84):
            return True
        else:
            return False

    def calculate_likelihood(self, x, y, t, z):
        """
        Return 0 for impossible value, None for unable to decide
        """

        # allow custom function to check if location is inside map
        if not self.inside_map(x,y):
            return 0

        min_m = None

        for wall in self.walls:
            a_x, a_y, b_x, b_y = wall

            # find if cos(beta) < min_sonar_angle_cos (too large angle)
            # BUG: also check direction??????????
            ab_distance = sqrt( (a_y-b_y)**2 + (b_x-a_x)**2 )
            beta = (cos(t)*(a_y-b_y)+sin(t)*(b_x-a_x))/ab_distance
            if beta < min_sonar_angle_cos:
                continue

            # find if distance is greater than possible to detect
            # BUG: also check direction?????????? (m<0)??
            m = ((b_y-a_y)*(a_x-x)-(b_x-a_x)*(a_y-y)) / ((b_y-a_y)*cos(t)-(b_x-a_x)*sin(t))
            if m > max_sonar_distance or m<0:
                continue

            # check if sonar is hit between endpoint
            _x = x + m*cos(t)
            _y = y + m*sin(t)
            if not ((a_x<=_x and _x<=b_x) or (b_x<=_x and _x<=a_x)):
                continue
            if not ((a_y<=_y and _y<=b_y) or (b_y<=_y and _y<=a_y)):
                continue

            # all check passed, add if is closer
            if min_m is None or m<min_m:
                min_m = m

        # find min_m for the most likely wall, cal likelihood
        # assume gaussian distribution
        if min_m:
            return exp( - (z-m)**2 / (2*sonar_var) ) + sonar_K
        else:
            return None




    def findwall(self,x,y,theta,dummy):
        alldis = []
        index = []
        for i in range(len(self.walls)):
            test = calDis(*self.walls[i],x,y,theta)
            print(test)
            if (test<0): # discard negative distance
                print("Negative test")
                continue
            elif (not self.check_within(x+test*cos(theta),y+test*sin(theta),theta,test)): # check intersection not within endpoints
                print("Not within endpoints ")
                continue
            else:
                print("Found within endpoints")
                alldis.append(test)
                index.append(i)
                print(alldis)
        result = np.array(alldis)
        return [result.min(),self.wall_list[index[result.argmin()]]]

    def check_within(self,x,y,theta,dis):
        for i in range(len(self.walls)):
            temp = self.walls[i]
            x_upper = temp[2]
            x_lowwer = temp[0]
            y_upper = temp[3]
            y_lowwer = temp[1]
            if (x_upper < x_lowwer):
                x_upper,x_lowwer = x_lowwer, x_upper
            if (y_upper < y_lowwer):
                y_upper,y_lowwer = y_lowwer, y_upper
            if (x>= x_lowwer and x<=x_upper and y>= y_lowwer and y<=y_upper):
                return True
        return False