#!/usr/bin/env python

################################################################################
# Copyright (C) 2013 by Jerome Maye                                            #
# jerome.maye@gmail.com                                                        #
#                                                                              #
# This program is free software; you can redistribute it and/or modify         #
# it under the terms of the Lesser GNU General Public License as published by  #
# the Free Software Foundation; either version 3 of the License, or            #
# (at your option) any later version.                                          #
#                                                                              #
# This program is distributed in the hope that it will be useful,              #
# but WITHOUT ANY WARRANTY; without even the implied warranty of               #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                #
# Lesser GNU General Public License for more details.                          #
#                                                                              #
# You should have received a copy of the Lesser GNU General Public License     #
# along with this program. If not, see <http://www.gnu.org/licenses/>.         #
################################################################################

import sys, roslib, rospy
from mv_cameras.srv import *

def setFramerate(serial, framerate):
  rospy.wait_for_service("/mv_cameras/" + serial + "/set_framerate")
  try:
    request = rospy.ServiceProxy("/mv_cameras/" + serial + "/set_framerate",
      SetFramerate)
    response = request(framerate)
    if response.response:
      print "Framerate for %s set to: %f" %(serial, framerate)
    else:
      print "Failed to set framerate for %s to: %f" %(serial, framerate)
      print "Reason: %s" % response.message
  except rospy.ServiceException, exception:
    print "SetFramerate request failed: %s" % exception

def usage():
  return "%s SERIAL FRAMERATE" % sys.argv[0]

if __name__ == "__main__":
  if len(sys.argv) == 3:
    serial = str(sys.argv[1])
    framerate = float(sys.argv[2])
  else:
    print usage()
    sys.exit(1)
  setFramerate(serial, framerate)
