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
    for i in range(len(response.serials)):
      print response.serials[i]
  except rospy.ServiceException, exception:
    print "GetCameras request failed: %s" % exception

if __name__ == "__main__":
  getCameras()
