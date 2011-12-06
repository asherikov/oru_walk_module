#!/usr/bin/python

# Remote control of walking module

import os
import sys
import time

from naoqi import ALProxy
from optparse import OptionParser


parser = OptionParser()
parser.add_option("-a", "--action", action="store", type="int", dest="nao_action", default=0)
parser.add_option("-b", "--broker-ip", action="store", type="string", dest="IP", default="10.0.0.22")
parser.add_option("-p", "--broker-port", action="store", type="int", dest="PORT", default=9559)
(options, args) = parser.parse_args();

print '----- Started'


try:
	walk_proxy = ALProxy("mpc_walk",options.IP, options.PORT)
except Exception,e:
	print "Error when creating proxy:"
	print str(e)
	exit(1)

print '----- Proxy was created'

while True:

	# select action
	if options.nao_action == 0:
		print "Please enter '0' to exit script"
		print "Please enter '1' to set stiffness to 1"
		print "Please enter '2' to set stiffness to 0"
		print "Please enter '3' to to initial position"
		print "Please enter '4' to stop"
		print "Please enter '5' to walk"
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
			walk_proxy.stop()
		elif nao_action == 5:
			walk_proxy.walk()
	except Exception,e:
		print "Execution of the action was failed."
		exit(1)


	# leave if requested
	if nao_action < 1 or nao_action > 5 or options.nao_action != 0:
		print '----- The script was stopped'
		break

exit (0)

