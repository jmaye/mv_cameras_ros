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
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <image_transport/image_transport.h>

#include <libmv/DriverBase/Include/mvDriverBaseEnums.h>

#include "base/Thread.h"
#include "base/Mutex.h"
#include "mv_cameras/SetExposure.h"
#include "mv_cameras/SetGain.h"
#include "mv_cameras/SetFramerate.h"

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
    /// Diagnose camera
    void diagnoseCamera(diagnostic_updater::DiagnosticStatusWrapper& status);
    /// Set exposure service
    bool setExposure(mv_cameras::SetExposure::Request& request,
      mv_cameras::SetExposure::Response& response);
    /// Set gain service
    bool setGain(mv_cameras::SetGain::Request& request,
      mv_cameras::SetGain::Response& response);
    /// Set framerate service
    bool setFramerate(mv_cameras::SetFramerate::Request& request,
      mv_cameras::SetFramerate::Response& response);
    /** @}
      */

    /** \name Protected members
      @{
      */
    /// ROS node handle
    ros::NodeHandle _nodeHandle;
    /// ROS image transport
    image_transport::ImageTransport _imageTransport;
    /// Image snappy publisher
    ros::Publisher _imageSnappyPublisher;
    /// Image raw publisher
    image_transport::Publisher _imageRawPublisher;
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
    /// Timeout
    int _timeoutMs;
    /// Last request number
    int _lastRequestNr;
    /// Retry timeout
    double _retryTimeout;
    /// Camera serial
    std::string _serial;
    /// Device version
    std::string _deviceVersion;
    /// Firmware version
    std::string _firmwareVersion;
    /// Device family
    std::string _family;
    /// Device product
    std::string _product;
    /// Device IP
    std::string _deviceIP;
    /// Device subnet mask
    std::string _deviceSubnetMask;
    /// Device MAC address
    std::string _deviceMACAddress;
    /// Device FPGA version
    std::string _deviceFPGAVersion;
    /// Diagnostic updater
    diagnostic_updater::Updater _updater;
    /// Frequency diagnostic for image acquisition and publishing
    std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> _imgFreq;
    /// Image minimum frequency
    double _imgMinFreq;
    /// Image maximum frequency
    double _imgMaxFreq;
    /// FPS tolerance between desired and actual
    double _fpsTolerance;
    /// Last frame software timestamp
    double _lastFrameSwTime;
    /// Last frame number
    int _lastFrameNumber;
    /// Missed frames count
    int _missedFramesCount;
    /// Last inter-frame software time
    double _lastInterFrameSwTime;
    /// Last inter-frame hardware time
    long _lastInterFrameHwTime;
    /// Last image height
    int _lastImageHeight;
    /// Last image width
    int _lastImageWidth;
    /// Last image gain
    float _lastImageGain;
    /// Last image exposure time
    int _lastExposureTime;
    /// Last image hardware timestamp
    long _lastImageHwTimestamp;
    /// Last image channel description
    std::string _lastImageChannelDesc;
    /// Last image channel count
    int _lastImageChannelCount;
    /// Last image bytes per pixel
    int _lastImageBytesPerPixel;
    /// Image gain setting
    double _gain;
    /// Exposure service
    ros::ServiceServer _setExposureService;
    /// Gain service
    ros::ServiceServer _setGainService;
    /// Framerate service
    ros::ServiceServer _setFramerateService;
    /** @}
      */

  };

}

#endif // CAMERA_NODE_H
