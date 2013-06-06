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

def getCameras():
  rospy.wait_for_service("/mv_cameras_manager/get_cameras")
  try:
    request = rospy.ServiceProxy("/mv_cameras_manager/get_cameras", GetCameras)
    response = request()
    return response.serials
  except rospy.ServiceException, exception:
    print "GetCameras request failed: %s" % exception
    return response.serials

def getExposure(serial):
  rospy.wait_for_service("/mv_cameras_manager/" + serial + "/get_exposure")
  try:
    request = rospy.ServiceProxy(
      "/mv_cameras_manager/" + serial + "/get_exposure", GetExposure)
    response = request()
    print "Exposure for %s is: %f" %(serial, response.exposure)
  except rospy.ServiceException, exception:
    print "GetExposure request failed for %s: %s" %(serial, exception)

if __name__ == "__main__":
  if len(sys.argv) == 2:
    serial = str(sys.argv[1])
    getExposure(serial)
  else:
    serials = getCameras()
    for i in range(len(serials)):
      getExposure(serials[i])
