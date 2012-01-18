#!/usr/bin/python
#-*- coding: iso-8859-15 -*-

# Remote control of walking module

import os
import sys
import time

from naoqi import ALProxy
from optparse import OptionParser


parser = OptionParser()
parser.add_option("-a", "--action", action="store", type="int", dest="nao_action", default=0)
parser.add_option("-b", "--broker-ip", action="store", type="string", dest="IP", default="127.0.0.1")
parser.add_option("-p", "--broker-port", action="store", type="int", dest="PORT", default=9559)
(options, args) = parser.parse_args();

print '----- Started'


try:
    walk_proxy = ALProxy("oru_walk", options.IP, options.PORT)
    motion_proxy = ALProxy("ALMotion", options.IP, options.PORT)
except Exception,e:
    print "Error when creating proxy:"
    print str(e)
    exit(1)

print '----- Proxy was created'

while True:

    # select action
    if options.nao_action == 0:
        print "Please enter a number corresponding to a command:"
        print "'0' - exit script"
        print "'1' - set stiffness to 1"
        print "'2' - set stiffness to 0"
        print "'3' - set initial position"
        print "'4' - stop"
        print "'5' - walk"
        print "'6' - set walk parameters"
        print "'7' - walk (using builtin module)"
        print "'8' - reset stiffness and angles (using builtin module)"
        try:
            nao_action = int (raw_input("Type a number: "))
        except Exception,e:
            print "Ooops!"
            exit(1)
    else:
        nao_action = options.nao_action


    # execute action
    try:
        if nao_action == 1:
            walk_proxy.setStiffness(1.0)
        elif nao_action == 2:
            walk_proxy.setStiffness(0.0)
        elif nao_action == 3:
            walk_proxy.initPosition()
        elif nao_action == 4:
            walk_proxy.stopWalking()
        elif nao_action == 5:
            walk_proxy.walk()
        elif nao_action == 6:
            # feedback_gain, feedback_threshold, mpc_alpha, mpc_beta, mpc_gamma, step_height
            walk_proxy.setWalkParameters(0.3, 0.006, 400.0, 8000.0, 1.0, 0.013)
        elif nao_action == 7:
            motion_proxy.stiffnessInterpolation("Body", 1.0, 0.1)
            motion_proxy.setWalkArmsEnabled(False, False)
            # enable motion whe lifted in the air
            motion_proxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", False]])
            motion_proxy.walkInit()

            # (X length, Y length, theta, frequency)
            motion_proxy.setWalkTargetVelocity(1.0, 0.0, 0.0, 1.0)
            time.sleep(4)

            motion_proxy.stopWalk()
        elif nao_action == 8:
            # reset stiffness and angles using motion proxy,
            # otherwise it doesn't work well later
            motion_proxy.stiffnessInterpolation("Body", 0.0, 1.0)
            numAngles = len(motion_proxy.getJointNames("Body"))
            angles = [0.0] * numAngles
            motion_proxy.angleInterpolationWithSpeed ("Body", angles, 0.3)


    except Exception,e:
        print "Execution of the action was failed."
        exit(1)


    # leave if requested
    if nao_action < 1 or nao_action > 8 or options.nao_action != 0:
        print '----- The script was stopped'
        break

exit (0)
