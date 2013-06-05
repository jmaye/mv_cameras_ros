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

def setGain(serial, gain):
  rospy.wait_for_service("/mv_cameras_manager/" + serial + "/set_gain")
  try:
    request = rospy.ServiceProxy(
      "/mv_cameras_manager/" + serial + "/set_gain", SetGain)
    response = request(gain)
    if response.response:
      print "Gain for %s set to: %f" %(serial, gain)
    else:
      print "Failed to set gain for %s to: %f" %(serial, gain)
      print "Reason: %s" % response.message
  except rospy.ServiceException, exception:
    print "SetGain request failed: %s" % exception

def usage():
  return "%s SERIAL GAIN" % sys.argv[0]

if __name__ == "__main__":
  if len(sys.argv) == 3:
    serial = str(sys.argv[1])
    gain = float(sys.argv[2])
  else:
    print usage()
    sys.exit(1)
  setGain(serial, gain)
