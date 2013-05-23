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

/** \file CameraNode.h
    \brief This file defines the CameraNode class which implements the
           Matrix Vision camera node.
  */

#ifndef CAMERA_NODE_H
#define CAMERA_NODE_H

#include <string>

#include <ros/ros.h>

#include <libmv/DriverBase/Include/mvDriverBaseEnums.h>

#include "base/Thread.h"
#include "base/Mutex.h"

namespace mvIMPACT {
  namespace acquire {
    class Device;
    template <typename T> class EnumPropertyI;
    class Request;
  }
}

namespace mv {

  /** The class CameraNode implements the Matrix Vision camera node.
      \brief Matrix Vision camera node
    */
  class CameraNode :
    public Thread {
  public:
    /** \name Types definitions
      @{
      */
    /// Self type
    typedef CameraNode Self;
    /** @}
      */

    /** \name Constructors/destructor
      @{
      */
    /// Constructor
    CameraNode(const ros::NodeHandle& nodeHandle,
      mvIMPACT::acquire::Device* device, bool isMaster = false);
    /// Copy constructor
    CameraNode(const Self& other) = delete;
    /// Copy assignment operator
    CameraNode& operator = (const Self& other) = delete;
    /// Move constructor
    CameraNode(Self&& other) = delete;
    /// Move assignment operator
    CameraNode& operator = (Self&& other) = delete;
    /// Destructor
    virtual ~CameraNode();
    /** @}
      */

    /** \name Accessors
      @{
      */
    /// Returns the serial of the device
    std::string getSerial() const;
    /// Returns the state of the device
    mvIMPACT::acquire::EnumPropertyI<TDeviceState> getState() const;
    /// Is this device master?
    bool isMaster() const;
    /** @}
      */

  protected:
    /** \name Protected methods
      @{
      */
    /// Main loop for the thread
    virtual void process();
    /// Sets synchronization
    void setSynchronization();
    /// Sets the features of the camera
    void setFeatures();
    /// Init the acquisition
    void initAcquisition();
    /// Retrieves the parameters
    void getParameters();
    /// Converts a MV MAC address to a readable string
    std::string getMACAddress(const std::string& mac) const;
    /// Converts a MV IP address to a readable string
    std::string getIPAddress(const std::string& ip) const;
    /// Publishes an image
    void publishImage(const ros::Time& timestamp,
      const mvIMPACT::acquire::Request* request);
    /** @}
      */

    /** \name Protected members
      @{
      */
    /// ROS node handle
    ros::NodeHandle _nodeHandle;
    /// Image snappy publisher
    ros::Publisher _imageSnappyPublisher;
    /// Frame ID
    std::string _frameId;
    /// Device handle
    mvIMPACT::acquire::Device* _device;
    /// Is this device master?
    bool _isMaster;
    /// Framerate [fps]
    double _framerate;
    /// Mutex protecting the camera
    mutable Mutex _mutex;
    /// Queue depth
    int _queueDepth;
    /// Exposure time of the camera in microseconds
    double _exposureTime;
    /// Width of the image
    int _width;
    /// Height of the image
    int _height;
    /// Request number
    int _requestNr;
    /// Counter
    unsigned int _cnt;
    /// Timeout
    int _timeoutMs;
    /// Last request number
    int _lastRequestNr;
    /// Retry timeout
    double _retryTimeout;
    /** @}
      */

  };

}

#endif // CAMERA_NODE_H
