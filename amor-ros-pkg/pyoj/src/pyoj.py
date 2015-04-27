#!/usr/bin/env python
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('joy')
import rospy
from joy.msg import Joy
import SocketServer
import math

PORTNO = 10552

class handler(SocketServer.DatagramRequestHandler):
    def handle(self):
        newmsg = self.rfile.readline().rstrip()
        print "Client %s said ``%s''" % (self.client_address[0], newmsg)
        self.wfile.write(self.server.oldmsg)
        self.server.oldmsg = newmsg
	args = newmsg.split(',')
	x = -float(args[2])
	y = float(args[3])
	z = float(args[4])
	d = math.sin(math.radians(5))

	if abs(x) < d:
	  x = 0
	else:
	  x -= x/math.fabs(x)*d

	if abs(z) < d:
	  z = 0
	else:
	  z -= z/math.fabs(z)*d

	k = 1 / math.cos(math.radians(60))
        pub.publish(Joy([1.5*k*z, 0.0, 0.0, k*x], [1] ))

pub = rospy.Publisher('joy', Joy)
rospy.init_node('pyoj', anonymous=True)
s = SocketServer.UDPServer(('',PORTNO), handler)
print "Awaiting UDP messages on port %d" % PORTNO
s.oldmsg = "This is the starting message."
s.serve_forever()

