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

#include "sensor/CameraNode.h"

#include <sstream>
#include <array>
#include <iomanip>
#include <algorithm>

#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire_GenICam.h>

#include <snappy.h>

#include "base/Timer.h"

#include "mv_cameras/ImageSnappyMsg.h"

using namespace mvIMPACT::acquire;
using namespace mvIMPACT::acquire::GenICam;

namespace mv {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  CameraNode::CameraNode(const ros::NodeHandle& nodeHandle, Device* device,
      bool isMaster) :
      _nodeHandle(nodeHandle),
      _device(device),
      _isMaster(isMaster),
      _requestNr(INVALID_ID),
      _cnt(0),
      _lastRequestNr(INVALID_ID) {
    getParameters();
    // setting interface to GenICAm, needed for setting synchronization
    _device->interfaceLayout.write(dilGenICam);
    // acquisition start/stop by user
    _device->acquisitionStartStopBehaviour.write(assbUser);
    _device->open();
    DeviceModule dm(_device);
    DeviceControl dc(_device);
    ROS_INFO_STREAM("CameraNode::CameraNode(): "
      << "InterfaceLayout: "
      << _device->interfaceLayout.readS() << std::endl
      << "gevDeviceIPAddress: " << getIPAddress(dm.gevDeviceIPAddress.readS())
      << std::endl
      << "gevDeviceSubnetMask: " << getIPAddress(dm.gevDeviceSubnetMask.readS())
      << std::endl
      << "gevDeviceMACAddress: "
      << getMACAddress(dm.gevDeviceMACAddress.readS()) << std::endl
      << "deviceTemperature: " << dc.deviceTemperature.read() << " [deg. C]"
      << std::endl
      << "mvDeviceFPGAVersion: " << dc.mvDeviceFPGAVersion.readS());
    ROS_INFO_STREAM("CameraNode::CameraNode(): opened device: "
      << _device->serial.read());
    setSynchronization();
    setFeatures();
    initAcquisition();
    ROS_INFO_STREAM("CameraNode::CameraNode(): acquisition starting: "
      << _device->serial.read());
    _imageSnappyPublisher = _nodeHandle.advertise<mv_cameras::ImageSnappyMsg>(
      _device->serial.readS() + "/image_snappy", _queueDepth);
  }

  CameraNode::~CameraNode() {
    FunctionInterface fi(_device);
    TDMR_ERROR result = DMR_NO_ERROR;
    if (_device->acquisitionStartStopBehaviour.read() == assbUser) {
      if ((result = static_cast<TDMR_ERROR>(fi.acquisitionStop())) !=
          DMR_NO_ERROR) {
        ROS_ERROR_STREAM("CameraNode::~CameraNode(): "
          << "'FunctionInterface.acquisitionStop' returned with an "
          "unexpected result: " << result
          << "(" << ImpactAcquireException::getErrorCodeAsString(result)
          << ")");
      }
    }
    if (fi.isRequestNrValid(_requestNr)) {
      fi.imageRequestUnlock(_requestNr);
    }
    fi.imageRequestReset(0, 0);
    while ((_requestNr = fi.imageRequestWaitFor(0)) >= 0) {
      ROS_ERROR_STREAM("CameraNode::~CameraNode(): "
        "Request " << _requestNr << " did return with status "
        << fi.getRequest(_requestNr)->requestResult.readS());
      fi.imageRequestUnlock(_requestNr);
    }
    _device->close();
    ROS_INFO_STREAM("CameraNode::~CameraNode(): closed device: "
      << _device->serial.read());
  }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

  std::string CameraNode::getSerial() const {
    Mutex::ScopedLock lock(_mutex);
    return _device->serial.read();
  }

  PropertyIDeviceState CameraNode::getState() const {
    Mutex::ScopedLock lock(_mutex);
    return _device->state;
  }

  bool CameraNode::isMaster() const {
    Mutex::ScopedLock lock(_mutex);
    return _isMaster;
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  std::string CameraNode::getIPAddress(const std::string& ip) const {
    std::string ipAddress = ip.substr(2);
    std::array<int, 4> ipAddressInt{{0, 0, 0, 0}};
    size_t idx = 3;
    for (int i = ipAddress.size() - 1; i >= 0; ) {
      std::string subStr;
      if (i - 1 >= 0) {
        subStr = ipAddress.substr(i - 1, 2);
        i -= 2;
      }
      else {
        subStr = ipAddress[i];
        i--;
      }
      ipAddressInt[idx--] = std::stoi(subStr, nullptr, 16);
    }
    std::stringstream ipAddressSs;
    ipAddressSs << std::dec
      << ipAddressInt[0];
    ipAddressSs << ".";
    ipAddressSs << std::dec
      << ipAddressInt[1];
    ipAddressSs << ".";
    ipAddressSs << std::dec
      << ipAddressInt[2];
    ipAddressSs << ".";
    ipAddressSs << std::dec
      << ipAddressInt[3];
    return ipAddressSs.str();
  }

  std::string CameraNode::getMACAddress(const std::string& mac) const {
    std::string deviceMACAddress = mac.substr(2);
    std::array<int, 6> deviceMACAddressInt{{0, 0, 0, 0, 0, 0}};
    size_t idx = 5;
    for (int i = deviceMACAddress.size() - 1; i >= 0; ) {
      std::string subStr;
      if (i - 1 >= 0) {
        subStr = deviceMACAddress.substr(i - 1, 2);
        i -= 2;
      }
      else {
        subStr = deviceMACAddress[i];
        i--;
      }
      deviceMACAddressInt[idx--] = std::stoi(subStr, nullptr, 16);
    }
    std::stringstream deviceMACAddressSs;
    deviceMACAddressSs << std::hex << std::setfill('0') << std::setw(2)
      << deviceMACAddressInt[0];
    deviceMACAddressSs << ":";
    deviceMACAddressSs << std::hex << std::setfill('0') << std::setw(2)
      << deviceMACAddressInt[1];
    deviceMACAddressSs << ":";
    deviceMACAddressSs << std::hex << std::setfill('0') << std::setw(2)
      << deviceMACAddressInt[2];
    deviceMACAddressSs << ":";
    deviceMACAddressSs << std::hex << std::setfill('0') << std::setw(2)
      << deviceMACAddressInt[3];
    deviceMACAddressSs << ":";
    deviceMACAddressSs << std::hex << std::setfill('0') << std::setw(2)
      << deviceMACAddressInt[4];
    deviceMACAddressSs << ":";
    deviceMACAddressSs << std::hex << std::setfill('0') << std::setw(2)
      << deviceMACAddressInt[5];
    return deviceMACAddressSs.str();
  }

  void CameraNode::publishImage(const ros::Time& timestamp,
      const Request* request) {
    mv_cameras::ImageSnappyMsgPtr imageSnappyMsg(
      new mv_cameras::ImageSnappyMsg);
    imageSnappyMsg->header.stamp = ros::Time::now();
    imageSnappyMsg->header.frame_id = _frameId;
    imageSnappyMsg->header.seq = request->infoFrameNr.read();
    imageSnappyMsg->width = request->imageWidth.read();
    imageSnappyMsg->height = request->imageHeight.read();
//    ROS_INFO_STREAM("CameraNode::publishImage(): "
//      "uncompressed size: " << request->imageSize.read());
    std::string imageSnappy;
    snappy::Compress(reinterpret_cast<char*>(request->imageData.read()),
      request->imageSize.read(), &imageSnappy);
//    ROS_INFO_STREAM("CameraNode::publishImage(): "
//      "compressed size: " << imageSnappy.size());
    imageSnappyMsg->data.resize(imageSnappy.size());
    std::copy(imageSnappy.begin(), imageSnappy.end(),
      imageSnappyMsg->data.begin());
    _imageSnappyPublisher.publish(imageSnappyMsg);
  }

  void CameraNode::process() {
    try {
      if (_device->state.read() == dsAbsent)
        return;
      Statistics statistics(_device);
      FunctionInterface fi(_device);
      _requestNr = fi.imageRequestWaitFor(_timeoutMs);
      if (fi.isRequestNrValid(_requestNr)) {
        const Request* pRequest = fi.getRequest(_requestNr);
        if (pRequest->isOK()) {
          ++_cnt;
          if (_cnt % 100 == 0) {
            ROS_INFO_STREAM("CameraNode::process(): "
              << "Info from " << _device->serial.readS()
              << ": " << statistics.framesPerSecond.name() << ": "
              << statistics.framesPerSecond.readS()
              << ", " << statistics.errorCount.name() << ": "
              << statistics.errorCount.readS()
              << ", " << statistics.captureTime_s.name() << ": "
              << statistics.captureTime_s.readS());
          }
          publishImage(ros::Time::now(), pRequest);
        }
        else {
          ROS_ERROR_STREAM("CameraNode::process(): "
            << "Error: " << pRequest->requestResult.readS());
        }
        if (fi.isRequestNrValid(_lastRequestNr)) {
          fi.imageRequestUnlock(_lastRequestNr);
        }
        _lastRequestNr = _requestNr;
        fi.imageRequestSingle();
      }
      else {
        ROS_ERROR_STREAM("CameraNode::process(): "
          << "imageRequestWaitFor failed (" << _requestNr << ", "
          << ImpactAcquireException::getErrorCodeAsString(_requestNr) << ")"
          << ", timeout value too small?");
      }
    }
    catch (ImpactAcquireException& e) {
      ROS_WARN_STREAM("CameraNode::process(): "
        "ImpactAcquireException: " << std::endl
        << "serial: " << _device->serial.readS()
        << "error code: " << e.getErrorCodeAsString() << std::endl
        << "message: " << e.what());
      ROS_WARN_STREAM("Retrying in " << _retryTimeout << " [s]");
      Timer timer;
      timer.sleep(_retryTimeout);
    }
  }

  void CameraNode::setSynchronization() {
    if (_isMaster) {
      // duration of a frame in microseconds for the requested framerate
      const double frameDuration = 1e6 / _framerate;
      CounterAndTimerControl catcMaster(_device);
      // timer1 determines the framerate
      catcMaster.timerSelector.writeS("Timer1");
      catcMaster.timerDelay.write(0.);
      catcMaster.timerDuration.write(frameDuration);
      catcMaster.timerTriggerSource.writeS("Timer1End");
      // timer2 triggers the integration start when timer1 ends
      catcMaster.timerSelector.writeS("Timer2");
      catcMaster.timerDelay.write(0.);
      catcMaster.timerDuration.write(10000.);
      catcMaster.timerTriggerSource.writeS("Timer1End");
      // set digital I/O for the synchronization
      DigitalIOControl io(_device);
      io.lineSelector.writeS("Line0");
      io.lineSource.writeS( "Timer2Active" );
      io.lineSelector.writeS("Line1");
      io.lineSource.writeS("Counter2Active");
    }
    // set trigger of master and slave camera, rising edge of timer2
    AcquisitionControl ac(_device);
    ac.triggerSelector.writeS("FrameStart");
    ac.triggerMode.writeS("On");
    ac.triggerSource.writeS("Line4");
    ac.triggerActivation.writeS("RisingEdge");
  }

  void CameraNode::setFeatures() {
    AcquisitionControl ac(_device);
    ac.exposureTime.write(_exposureTime);
    ImageFormatControl ifc(_device);
    ImageProcessing ip(_device);
    // TODO: make this a configurable parameter
    ifc.pixelFormat.writeS("BayerGR8");
    ip.colorProcessing.write(cpmBayerToMono);
    ifc.width.write(_width);
    ifc.height.write(_height);
  }

  void CameraNode::initAcquisition() {
    FunctionInterface fi(_device);
    TDMR_ERROR result = DMR_NO_ERROR;
    while ((result = static_cast<TDMR_ERROR>(fi.imageRequestSingle())) ==
        DMR_NO_ERROR) {
    };
    if (result != DEV_NO_FREE_REQUEST_AVAILABLE) {
      ROS_ERROR_STREAM("CameraNode::initAcquisition(): " << _device->serial
        << "'FunctionInterface.imageRequestSingle' returned with an "
        "unexpected result: " << result
        << "(" << ImpactAcquireException::getErrorCodeAsString(result)
        << ")");
    }
    if (_device->acquisitionStartStopBehaviour.read() == assbUser) {
      if ((result = static_cast<TDMR_ERROR>(fi.acquisitionStart())) !=
          DMR_NO_ERROR) {
        ROS_ERROR_STREAM("CameraNode::initAcquisition(): " << _device->serial
          << "'FunctionInterface.acquisitionStart' returned with an "
          "unexpected result: " << result
          << "(" << ImpactAcquireException::getErrorCodeAsString(result)
          << ")");
      }
    }
  }

  void CameraNode::getParameters() {
    _nodeHandle.param<double>(_device->serial.readS() + "/framerate",
      _framerate, 10.0);
    _nodeHandle.param<double>(_device->serial.readS() + "/exposure_time",
      _exposureTime, 1000.0);
    _nodeHandle.param<int>(_device->serial.readS() + "/width",
      _width, 1280);
    _nodeHandle.param<int>(_device->serial.readS() + "/height",
      _height, 960);
    _nodeHandle.param<int>(_device->serial.readS() + "/timeout_ms",
      _timeoutMs, 500);
    _nodeHandle.param<double>(_device->serial.readS() + "/retry_timeout",
      _retryTimeout, 1);
    _nodeHandle.param<std::string>(_device->serial.readS() +
      "/frame_id", _frameId, _device->serial.readS());
    _nodeHandle.param<int>(_device->serial.readS() + "/queue_depth",
      _queueDepth, 100);
  }

}
