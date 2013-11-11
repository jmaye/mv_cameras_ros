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

import sys
import rospy
from mv_cameras.srv import *

def getCameras():
  rospy.wait_for_service("/mv_cameras_manager/get_cameras")
  try:
    request = rospy.ServiceProxy("/mv_cameras_manager/get_cameras", GetCameras)
    response = request()
    return response.serials
  except rospy.ServiceException, exception:
    print "GetCameras request failed: %s" % exception
    return response.serials

def setTimestampReset(serial, timestampReset):
  rospy.wait_for_service(
    "/mv_cameras_manager/" + serial + "/set_timestamp_reset")
  try:
    request = rospy.ServiceProxy(
      "/mv_cameras_manager/" + serial + "/set_timestamp_reset",
      SetTimestampReset)
    response = request(timestampReset)
    if response.response:
      print "Timestamp reset for %s set to: %f" %(serial, timestampReset)
    else:
      print "Failed to set timestamp reset for %s to: %f" %(serial,
        timestampReset)
      print "Reason: %s" % response.message
  except rospy.ServiceException, exception:
    print "SetTimestampReset request failed for %s: %s" %(serial, exception)

def usage():
  return "%s SERIAL TIMESTAMP_RESET or TIMESTAMP_RESET" % sys.argv[0]

if __name__ == "__main__":
  if len(sys.argv) == 3:
    serial = str(sys.argv[1])
    timestampReset = int(sys.argv[2])
    setTimestampReset(serial, timestampReset)
  elif len(sys.argv) == 2:
    timestampReset = int(sys.argv[1])
    serials = getCameras()
    for i in range(len(serials)):
      setTimestampReset(serials[i], timestampReset)
  else:
    print usage()
    sys.exit(1)
