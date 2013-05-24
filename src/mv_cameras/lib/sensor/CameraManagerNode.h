/******************************************************************************
 * Copyright (C) 2013 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

/** \file CameraManagerNode.h
    \brief This file defines the CameraManagerNode class which implements the
           Matrix Vision camera manager node.
  */

#ifndef CAMERA_MANAGER_NODE_H
#define CAMERA_MANAGER_NODE_H

#include <memory>
#include <string>
#include <unordered_map>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

namespace mvIMPACT {
  namespace acquire {
    class DeviceManager;
  }
}

namespace mv {

  class CameraNode;

  /** The class CameraManagerNode implements the Matrix Vision camera manager
      node.
      \brief Matrix Vision camera manager node
    */
  class CameraManagerNode {
  public:
    /** \name Types definitions
      @{
      */
    /// Self type
    typedef CameraManagerNode Self;
    /** @}
      */

    /** \name Constructors/destructor
      @{
      */
    /// Constructor
    CameraManagerNode(const ros::NodeHandle& nh);
    /// Copy constructor
    CameraManagerNode(const Self& other) = delete;
    /// Copy assignment operator
    CameraManagerNode& operator = (const Self& other) = delete;
    /// Move constructor
    CameraManagerNode(Self&& other) = delete;
    /// Move assignment operator
    CameraManagerNode& operator = (Self&& other) = delete;
    /// Destructor
    virtual ~CameraManagerNode();
    /** @}
      */

    /** \name Methods
      @{
      */
    /// Spin
    int spin();
    /** @}
      */

  protected:
    /** \name Protected methods
      @{
      */
    /// Timely detects new cameras
    void updateDeviceList(const ros::TimerEvent& event);
    /// Retrieves the parameters
    void getParameters();
    /// Diagnose camera manager
    void diagnoseCameraManager(diagnostic_updater::DiagnosticStatusWrapper&
      status);
    /** @}
      */

    /** \name Protected members
      @{
      */
    /// ROS node handle
    ros::NodeHandle _nodeHandle;
    /// Matrix Vision device manager
    std::shared_ptr<mvIMPACT::acquire::DeviceManager> _deviceManager;
    /// Time in seconds between consecutive polling for devices
    double _updateDeviceListTime;
    /// Serial number of the master camera for synchronization
    std::string _masterCameraSerial;
    /// Container for camera nodes
    std::unordered_map<std::string, std::shared_ptr<CameraNode> > _cameraNodes;
    /// MV driver version string
    std::string _driverVersion;
    /// Diagnostic updater
    diagnostic_updater::Updater _updater;
    /// Frequency checker for the discover of devices
    std::shared_ptr<diagnostic_updater::FrequencyStatus> _cmFreq;
    /// Discover devices minimum frequency
    double _discoverMinFreq;
    /// Discover devices maximum frequency
    double _discoverMaxFreq;
    /** @}
      */

  };

}

#endif // CAMERA_MANAGER_NODE_H
