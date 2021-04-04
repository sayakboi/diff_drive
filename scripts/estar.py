#! /usr/bin/env python2


import rospy

from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

from std_srvs.srv import *

import numpy as np
import math
import random
import future


position_ = Point()


def clbk_odom(msg):  #####odometry info###
    global position_, yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def clbk_laser(msg):  ###laser output###
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }


def startPotentialSet(plane0):
    # setting rows with decreasing potentials(but equipotential row cells) after the bot's row
    for e in range(64):
        for f in range(64):
            plane0[e][f] = 64 - e
    return plane0


def readBotid():
    x = 0
    y = 0
    # Take x and y coordinates, find mod with 0.5 meters to get x and y coordinates
    bid = np.array([x, y])
    return bid


def readObid():
    oid = np.zeros(8, 2)
    x = 0
    y = 0
    for i in range(8):
        # Take x and y coordinates, find mod with 0.5 meters to get x and y coordinates
        oid[i] = [x, y]
    return oid


def Nearcells(botid):
    # make all members of the array as -5
    a = [[-5, -5], [-5, -5], [-5, -5], [-5, -5], [-5, -5], [-5, -5], [-5, -5], [-5, -5]]
    x = botid[0]
    y = botid[1]

    if x + 1 < 64:
        a[1][0] = x + 1
        a[1][1] = y
        if y + 1 < 64:
            a[2][0] = x + 1
            a[2][1] = y + 1
        if y - 1 > -1:
            a[3][0] = x + 1
            a[3][1] = y - 1
    if x - 1 > -1:
        a[4][0] = x - 1
        a[4][1] = y
        if y + 1 < 64:
            a[5][0] = x - 1
            a[5][1] = y - 1
        if y - 1 > -1:
            a[6][0] = x - 1
            a[6][1] = y - 1
    if y + 1 < 64:
        a[7][0] = x
        a[7][1] = y + 1
    if y - 1 > -1:
        a[8][0] = x
        a[8][1] = y - 1

    return a


def forbiddenAssign(obid, plane0):
    for i in range(8):
        plane0[(obid[i][0])][(obid[i][1])] = -1  # Here only the obstacle cells are assigned -1 potential
    for j in range(8):
        nearforbidden = Nearcells(obid[j]);
        for k in range(8):
            plane0[(nearforbidden[k][0])][
                (nearforbidden[k][1])] = -1  # Here the cells near to the obstacle cells are assigned -1 potential
    return plane0


def retcost(a, b):
    if a[0] > 0 and a[1] > 0 and b[0] > 0 and b[1] > 0:
        dist = math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)  # Absolute distance
        angle1 = math.atan2((b[0] - a[0]), (b[1] - a[1]))  # Angle between 0 and pi(negative also possible)
        # include code for getting orientation of bot
        angle = 0  # Get absolute difference between bot and the destination
        distCost = 5  # Distance cost
        angleCost = 5  # Turning cost
        netcost = dist * distCost + angle * angleCost  # Net cost
        return netcost
    else:
        return -1


def eligiblecells(plane0, a):  # Returns 1 if atleast one of the cells indexed by a in plane0 have positive potential
    x = -1
    for i in range(8):
        if plane0[a[i][0]][a[i][1]] > 0:
            x = -1
    if x > 0:
        return 1
    else:
        return -1


def updateWaypoint0(waypoint, botid, nearcells, plane0):
    if plane0[(botid[0])][(botid[1])] > 0:
        upcell = [-1, -1], downcell = [-1, -1]
        if plane0[(botid[0])][(botid[1] + 1)] > 0:
            upcell[0] = botid[0]
            upcell[1] = botid[1] + 1
        if plane0[(botid[0])][(botid[1] - 1)] > 0:
            downcell[0] = botid[0]
            downcell[1] = botid[1] - 1

        upcost = retcost(botid, upcell)
        downcost = retcost(botid, downcell)

        if upcost >= 0 and downcost >= 0:
            if upcost <= downcost:
                waypoint = upcell
            else:
                waypoint = downcell
        elif downcost < 0 and upcost > 0:
            waypoint = upcell
        elif downcost > 0 and upcost < 0:
            waypoint = downcell

    else:
        if eligiblecells(plane0, nearcells) > 0:
            nearcells[0] = waypoint
            for i in range(1, 8):
                if plane0[nearcells[i][0]][nearcells[i][1]] > plane0[waypoint[0]][waypoint[1]]:
                    waypoint = nearcells[i]
                else:
                    if plane0[nearcells[i][0]][nearcells[i][1]] == plane0[waypoint[0]][waypoint[1]]:
                        d1 = retcost(botid, nearcells[i])
                        d2 = retcost(botid, waypoint)
                        if d1 < d2:
                            waypoint = nearcells[i]
        else:
            if plane0[waypoint[0]][waypoint[1]] > 0:
                pass
            else:
                if plane0[waypoint[0]][waypoint[1]] <= 0:
                    waypoint[0] = -5
                    waypoint[1] = -5
    return waypoint


def movethere(waypoint):
    old_err = 0.0
    cur_err = 0.0
    goal = [0.5 * waypoint[0] + 0.25, 0.5 * waypoint[1] + 0.25]
    # bug2 algo here
    a = 5


def task(botid, plane0):
    plane = [botid[0]][botid[0]] = 0
    # insert code to clean the grid
    return plane0


def updateWaypoint2(plane0):
    plane2 = np.zeros(16, 16)
    for i in range(16):  # finding the average of the grids to put back in upper grid
        for j in range(16):
            x = 0
            x += plane0[4 * i][4 * j]
            x += plane0[4 * i][4 * j + 1]
            x += plane0[4 * i][4 * j + 2]
            x += plane0[4 * i][4 * j + 3]
            x += plane0[4 * i + 1][4 * j]
            x += plane0[4 * i + 1][4 * j + 1]
            x += plane0[4 * i + 1][4 * j + 2]
            x += plane0[4 * i + 1][4 * j + 3]
            x += plane0[4 * i + 2][4 * j]
            x += plane0[4 * i + 2][4 * j + 1]
            x += plane0[4 * i + 2][4 * j + 2]
            x += plane0[4 * i + 2][4 * j + 3]
            x += plane0[4 * i + 3][4 * j]
            x += plane0[4 * i + 3][4 * j + 1]
            x += plane0[4 * i + 3][4 * j + 2]
            x += plane0[4 * i + 3][4 * j + 3]

            x = x / 16
            plane2[i][j] = x

    max_potential = plane2.max()  # Gives the maximum potential among the grids
    a = np.array([-5, -5])
    for k in range(16):
        for l in range(32):
            if plane2[k][l] == max_potential:
                srand = random.randint(0, 16)  # selecting a base waypoint randomly
                if srand == 0:
                    a = [4 * k, 4 * l]
                elif srand == 1:
                    a = [4 * k, 4 * l + 1]
                elif srand == 2:
                    a = [4 * k, 4 * l + 2]
                elif srand == 3:
                    a = [4 * k, 4 * l + 3]
                elif srand == 4:
                    a = [4 * k + 1, 4 * l]
                elif srand == 5:
                    a = [4 * k + 1, 4 * l + 1]
                elif srand == 6:
                    a = [4 * k + 1, 4 * l + 2]
                elif srand == 7:
                    a = [4 * k + 1, 4 * l + 3]
                elif srand == 8:
                    a = [4 * k + 2, 4 * l]
                elif srand == 9:
                    a = [4 * k + 2, 4 * l + 1]
                elif srand == 10:
                    a = [4 * k + 2, 4 * l + 2]
                elif srand == 11:
                    a = [4 * k + 2, 4 * l + 3]
                elif srand == 12:
                    a = [4 * k + 3, 4 * l]
                elif srand == 13:
                    a = [4 * k + 3, 4 * l + 1]
                elif srand == 14:
                    a = [4 * k + 3, 4 * l + 2]
                else:
                    a = [4 * k + 3, 4 * l + 3]

    return a


def updateWaypoint1(plane0):
    plane1 = np.zeros(32, 32)
    for i in range(32):  # finding the average of the grids to put back in upper grid
        for j in range(32):
            x = 0
            x += plane0[2 * i][2 * j]
            x += plane0[2 * i][2 * j + 1]
            x += plane0[2 * i + 1][2 * j]
            x += plane0[2 * i + 1][2 * j + 1]
            x = x / 4
            plane1[i][j] = x

    max_potential = plane1.max()  # Gives the maximum potential among the grids
    a = np.array([-5, -5])
    for k in range(32):
        for l in range(32):
            if plane1[k][l] == max_potential:
                srand = random.randint(0, 4)  # selecting a base waypoint randomly
                if srand == 0:
                    a = [2 * k, 2 * l]
                elif srand == 1:
                    a = [2 * k, 2 * l + 1]
                elif srand == 2:
                    a = [2 * k + 1, 2 * l]
                else:
                    a = [2 * k + 1, 2 * l + 1]
    return a


def main():
    rospy.init_node('estar')

    sub_laser = rospy.Subscriber('/Diff_Drive/laser/scan1', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/Diff_Drive/diff_drive_controller/odom', Odometry, clbk_odom)

    current_state = 0  # 0=ST,1=CP0, 2=WT,3=CP1,4=CP2, 5=FN
    plane0 = np.zeros(64, 64)  # initialising all zeroes
    botid = np.zeros(2)  # bot id
    obid = np.zeros(8, 2)  # object id considering 8  objects

    while not rospy.is_shutdown():
        botid = readBotid()
        obid = readObid()

        if current_state == 0:
            waypoint = botid
            plane0 = startPotentialSet(plane0)
            current_state = 1
        elif current_state == 1:
            plane0 = forbiddenAssign(obid, plane0)
            nearcells = Nearcells(botid)
            waypoint = updateWaypoint0(waypoint, botid, nearcells, plane0)
            a = np.array([-5, -5])
            if waypoint != botid and waypoint != a:
                movethere(waypoint)
                current_state = 1
            if waypoint == botid:
                movethere(waypoint)
                current_state = 2
            if waypoint == a:
                current_state = 3
        elif current_state == 2:
            plane0 = task(botid, plane0);  # gives back plane0 with the
            current_state = 1
        elif current_state == 3:
            waypoint = updateWaypoint1(plane0)
            a = np.array([-5, -5])
            if waypoint != a:
                current_state = 1
            else:
                current_state = 4
        elif current_state == 4:
            waypoint = updateWaypoint2(plane0)
            if waypoint != a:
                current_state = 1
            else:
                current_state = 5
        elif current_state == 5:
            print('Coverage Complete')
