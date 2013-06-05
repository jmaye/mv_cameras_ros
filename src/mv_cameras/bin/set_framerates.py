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

def setFramerates(framerate):
  rospy.wait_for_service("/mv_cameras_manager/set_framerates")
  try:
    request = rospy.ServiceProxy("/mv_cameras_manager/set_framerates",
      SetFramerate)
    response = request(framerate)
    if response.response:
      print "Framerates set to: %f" %(framerate)
    else:
      print "Failed to set framerates to: %f" %(framerate)
      print "Reason: %s" % response.message
  except rospy.ServiceException, exception:
    print "SetFramerate request failed: %s" % exception

def usage():
  return "%s FRAMERATE" % sys.argv[0]

if __name__ == "__main__":
  if len(sys.argv) == 2:
    framerate = float(sys.argv[1])
  else:
    print usage()
    sys.exit(1)
  setFramerates(framerate)
