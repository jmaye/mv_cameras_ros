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

#include <cmath>

#include <sstream>
#include <array>
#include <iomanip>
#include <algorithm>

#include <mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire_GenICam.h>

#include <snappy.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "base/Timer.h"
#include "base/Timestamp.h"

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
      _imageTransport(nodeHandle),
      _device(device),
      _isMaster(isMaster),
      _requestNr(INVALID_ID),
      _lastRequestNr(INVALID_ID),
      _lastFrameSwTime(0),
      _lastFrameNumber(0),
      _missedFramesCount(0),
      _lastInterFrameSwTime(0),
      _lastInterFrameHwTime(0),
      _lastImageHeight(0),
      _lastImageWidth(0),
      _lastImageGain(0),
      _lastExposureTime(0),
      _lastImageHwTimestamp(0),
      _lastImageChannelCount(0),
      _lastImageBytesPerPixel(0) {
    getParameters();
    _serial = _device->serial.readS();
    _deviceVersion = _device->deviceVersion.readS();
    _firmwareVersion = _device->firmwareVersion.getTranslationDictString();
    _family = _device->family.readS();
    _product = _device->product.readS();
    // setting interface to GenICAm, needed for setting synchronization
    _device->interfaceLayout.write(dilGenICam);
    // acquisition start/stop by user
    _device->acquisitionStartStopBehaviour.write(assbUser);
    _device->open();
    DeviceModule dm(_device);
    DeviceControl dc(_device);
    _deviceIP = getIPAddress(dm.gevDeviceIPAddress.readS());
    _deviceSubnetMask = getIPAddress(dm.gevDeviceSubnetMask.readS());
    _deviceMACAddress = getMACAddress(dm.gevDeviceMACAddress.readS());
    _deviceFPGAVersion = dc.mvDeviceFPGAVersion.readS();
    setSynchronization();
    setFeatures();
    initAcquisition();
    _updater.setHardwareID(_serial);
    _imageSnappyPublisher =
      _nodeHandle.advertise<mv_cameras::ImageSnappyMsg>(
      _serial + "/image_snappy", _queueDepth);
    _imgFreq.reset(new diagnostic_updater::HeaderlessTopicDiagnostic(
      _serial + "/image_snappy", _updater,
      diagnostic_updater::FrequencyStatusParam(&_imgMinFreq, &_imgMaxFreq,
      0.1, 10)));
    _imageRawPublisher = _imageTransport.advertise(_serial + "/image_raw",
      _queueDepth);
    _setExposureService = _nodeHandle.advertiseService(
      _serial + "/set_exposure", &CameraNode::setExposure, this);
    _setGainService = _nodeHandle.advertiseService(_serial + "/set_gain",
      &CameraNode::setGain, this);
    _setFramerateService = _nodeHandle.advertiseService(
      _serial + "/set_framerate", &CameraNode::setFramerate, this);
    _updater.add(_serial + " Camera status", this, &CameraNode::diagnoseCamera);
    _updater.force_update();
  }

  CameraNode::~CameraNode() {
    FunctionInterface fi(_device);
    TDMR_ERROR result = DMR_NO_ERROR;
    if (_device->acquisitionStartStopBehaviour.read() == assbUser) {
      if ((result = static_cast<TDMR_ERROR>(fi.acquisitionStop())) !=
          DMR_NO_ERROR) {
        ROS_WARN_STREAM("CameraNode::~CameraNode(): "
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
      ROS_WARN_STREAM("CameraNode::~CameraNode(): "
        "Request " << _requestNr << " did return with status "
        << fi.getRequest(_requestNr)->requestResult.readS());
      fi.imageRequestUnlock(_requestNr);
    }
    _device->close();
  }

/******************************************************************************/
/* Accessors                                                                  */
/******************************************************************************/

  std::string CameraNode::getSerial() const {
    Mutex::ScopedLock lock(_mutex);
    return _serial;
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
      ipAddressInt[idx--] = std::stoi(subStr, 0, 16);
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
      deviceMACAddressInt[idx--] = std::stoi(subStr, 0, 16);
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
    imageSnappyMsg->header.stamp = timestamp;
    imageSnappyMsg->header.frame_id = _frameId;
    imageSnappyMsg->header.seq = request->infoFrameNr.read();
    imageSnappyMsg->width = request->imageWidth.read();
    imageSnappyMsg->height = request->imageHeight.read();
    imageSnappyMsg->hwTimestamp = request->infoTimeStamp_us.read();
    std::string imageSnappy;
    snappy::Compress(reinterpret_cast<char*>(request->imageData.read()),
      request->imageSize.read(), &imageSnappy);
    imageSnappyMsg->data.resize(imageSnappy.size());
    std::copy(imageSnappy.begin(), imageSnappy.end(),
      imageSnappyMsg->data.begin());
    _imageSnappyPublisher.publish(imageSnappyMsg);
    sensor_msgs::ImagePtr imageRawMsg(new sensor_msgs::Image);
    imageRawMsg->header.stamp = timestamp;
    imageRawMsg->header.frame_id = _frameId;
    imageRawMsg->header.seq = request->infoFrameNr.read();
    imageRawMsg->height = request->imageHeight.read();
    imageRawMsg->width = request->imageWidth.read();
    imageRawMsg->step = request->imageLinePitch.read();
    imageRawMsg->encoding = sensor_msgs::image_encodings::MONO8;
    imageRawMsg->data.resize(request->imageSize.read());
    std::copy(reinterpret_cast<char*>(request->imageData.read()),
      reinterpret_cast<char*>(request->imageData.read()) +
      request->imageSize.read(),
      imageRawMsg->data.begin());
    _imageRawPublisher.publish(imageRawMsg);
    _imgFreq->tick();
    _updater.update();
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
          const double acqTime = Timestamp::now();
          _lastImageWidth = pRequest->imageWidth.read();
          _lastImageHeight = pRequest->imageHeight.read();
          if (_lastFrameNumber &&
              (pRequest->infoFrameNr.read() != _lastFrameNumber + 1))
            _missedFramesCount++;
          _lastFrameNumber = pRequest->infoFrameNr.read();
          if (_lastFrameSwTime)
            _lastInterFrameSwTime = acqTime - _lastFrameSwTime;
          _lastFrameSwTime = acqTime;
          _lastImageGain = pRequest->infoGain_dB.read();
          _lastExposureTime = pRequest->infoExposeTime_us.read();
          _lastImageChannelDesc = pRequest->imageChannelDesc.read();
          _lastImageChannelCount = pRequest->imageChannelCount.read();
          _lastImageBytesPerPixel = pRequest->imageBytesPerPixel.read();
          const long hwTimestamp = pRequest->infoTimeStamp_us.read();
          if (_lastImageHwTimestamp)
            _lastInterFrameHwTime = hwTimestamp - _lastImageHwTimestamp;
          _lastImageHwTimestamp = hwTimestamp;
          publishImage(ros::Time(acqTime), pRequest);
        }
        else {
          ROS_WARN_STREAM("CameraNode::process(): "
            << "Error: " << pRequest->requestResult.readS());
        }
        if (fi.isRequestNrValid(_lastRequestNr)) {
          fi.imageRequestUnlock(_lastRequestNr);
        }
        _lastRequestNr = _requestNr;
        fi.imageRequestSingle();
      }
      else {
        ROS_WARN_STREAM("CameraNode::process(): "
          << "imageRequestWaitFor failed (" << _requestNr << ", "
          << ImpactAcquireException::getErrorCodeAsString(_requestNr) << ")"
          << ", timeout value too small?");
      }
    }
    catch (const ImpactAcquireException& e) {
      ROS_WARN_STREAM("CameraNode::process(): "
        "ImpactAcquireException: " << std::endl
        << "serial: " << _device->serial.readS() << std::endl
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
      catcMaster.timerDuration.write(1000.);
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
    AnalogControl anc(_device);
    anc.gain.write(_gain);
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
      _exposureTime, 10000.0);
    if (_exposureTime > 1e6 / _framerate)
      ROS_WARN_STREAM("CameraNode::getParameters(): "
        "exposure time is bigger than frame duration");
    _nodeHandle.param<double>(_device->serial.readS() + "/gain",
      _gain, 0.0);
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
    _nodeHandle.param<double>(_device->serial.readS() + "/fps_tolerance",
      _fpsTolerance, 2);
    _nodeHandle.param<double>(_device->serial.readS() + "/img_min_freq",
      _imgMinFreq, _framerate - _fpsTolerance);
    _nodeHandle.param<double>(_device->serial.readS() + "/img_max_freq",
      _imgMaxFreq, _framerate + _fpsTolerance);
  }

  void CameraNode::diagnoseCamera(diagnostic_updater::DiagnosticStatusWrapper&
      status) {
    Mutex::ScopedLock lock(_mutex);
    DeviceControl dc(_device);
    float temperature = dc.deviceTemperature.read();
    Statistics statistics(_device);
    float captureTime_s = statistics.captureTime_s.read();
    int errorCount = statistics.errorCount.read();
    int abortedRequestsCount = statistics.abortedRequestsCount.read();
    int timedOutRequestsCount = statistics.timedOutRequestsCount.read();
    float framesPerSecond = statistics.framesPerSecond.read();
    int frameCount = statistics.frameCount.read();
    float imageProcTime_s = statistics.imageProcTime_s.read();
    float formatConvertTime_s = statistics.formatConvertTime_s.read();
    float queueTime_s = statistics.queueTime_s.read();
    int lostImagesCount = statistics.lostImagesCount.read();
    int framesIncompleteCount = statistics.framesIncompleteCount.read();
    float missingDataAverage_pc = statistics.missingDataAverage_pc.read();
    long retransmitCount = statistics.retransmitCount.read();
    status.add("Serial", _serial);
    status.add("Version", _deviceVersion);
    status.add("Firmware", _firmwareVersion);
    status.add("FPGA", _deviceFPGAVersion);
    status.add("Family", _family);
    status.add("Product", _product);
    status.add("IP address", _deviceIP);
    status.add("Subnet mask", _deviceSubnetMask);
    status.add("MAC address", _deviceMACAddress);
    status.add("State", _device->state.readS());
    status.add("Temperature", temperature);
    status.add("Capture time", captureTime_s);
    status.add("Error count", errorCount);
    status.add("Aborted requests count", abortedRequestsCount);
    status.add("Timed out requests count", timedOutRequestsCount);
    status.add("Frames per second", framesPerSecond);
    status.add("Frame count", frameCount);
    status.add("Image processing time", imageProcTime_s);
    status.add("Format convert time", formatConvertTime_s);
    status.add("Queue time", queueTime_s);
    status.add("Lost images count", lostImagesCount);
    status.add("Frames incomplete count", framesIncompleteCount);
    status.add("Missing data average", missingDataAverage_pc);
    status.add("Retransmit count", retransmitCount);
    status.add("Missed frames count", _missedFramesCount);
    status.add("Image height", _lastImageHeight);
    status.add("Image width", _lastImageWidth);
    status.add("Image gain [db]", _lastImageGain);
    status.add("Image exposure time [us]", _lastExposureTime);
    status.addf("Image software timestamp [s]", "%f", _lastFrameSwTime);
    status.add("Image hardware timestamp [us]", _lastImageHwTimestamp);
    status.add("Inter-frame software time [s]", _lastInterFrameSwTime);
    status.add("Inter-frame hardware time [us]", _lastInterFrameHwTime);
    status.add("Image channel description", _lastImageChannelDesc);
    status.add("Image channel count", _lastImageChannelCount);
    status.add("Image bytes per pixel", _lastImageBytesPerPixel);
    if (std::fabs(_framerate - framesPerSecond) < _fpsTolerance)
      status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
        "Target framerate met (desired: %f, actual: %f).",
        _framerate, framesPerSecond);
    else
      status.summaryf(diagnostic_msgs::DiagnosticStatus::WARN,
        "Target framerate not met (desired: %f, actual: %f).",
        _framerate, framesPerSecond);
  }

  bool CameraNode::setExposure(mv_cameras::SetExposure::Request& request,
      mv_cameras::SetExposure::Response& response) {
    Mutex::ScopedLock lock(_mutex);
    try {
      _exposureTime = request.exposure;
      AcquisitionControl ac(_device);
      ac.exposureTime.write(_exposureTime);
      if (_exposureTime > 1e6 / _framerate) {
        ROS_WARN_STREAM("CameraNode::setExposure(): "
          "exposure time is bigger than frame duration");
        response.response = true;
        response.message = "Exposure time is bigger than frame duration";
      }
      else {
        response.response = true;
        response.message = "Success";
      }
    }
    catch (const ImpactAcquireException& e) {
      ROS_WARN_STREAM("CameraNode::setExposure(): "
        "ImpactAcquireException: " << std::endl
        << "serial: " << _device->serial.readS() << std::endl
        << "error code: " << e.getErrorCodeAsString() << std::endl
        << "message: " << e.what());
      response.response = false;
      response.message = e.what();
    }
    return true;
  }

  bool CameraNode::setGain(mv_cameras::SetGain::Request& request,
      mv_cameras::SetGain::Response& response) {
    Mutex::ScopedLock lock(_mutex);
    try {
      _gain = request.gain;
      AnalogControl anc(_device);
      anc.gain.write(_gain);
      response.response = true;
      response.message = "Success";
    }
    catch (const ImpactAcquireException& e) {
      ROS_WARN_STREAM("CameraNode::setGain(): "
        "ImpactAcquireException: " << std::endl
        << "serial: " << _device->serial.readS() << std::endl
        << "error code: " << e.getErrorCodeAsString() << std::endl
        << "message: " << e.what());
      response.response = false;
      response.message = e.what();
    }
    return true;
  }

  bool CameraNode::setFramerate(mv_cameras::SetFramerate::Request& request,
      mv_cameras::SetFramerate::Response& response) {
    Mutex::ScopedLock lock(_mutex);
    try {
      _framerate = request.framerate;
      const double frameDuration = 1e6 / _framerate;
      CounterAndTimerControl catcMaster(_device);
      catcMaster.timerSelector.writeS("Timer1");
      catcMaster.timerDuration.write(frameDuration);
      if (_exposureTime > 1e6 / _framerate) {
        ROS_WARN_STREAM("CameraNode::setFramerate(): "
          "exposure time is bigger than frame duration");
        response.response = true;
        response.message = "Exposure time is bigger than frame duration";
      }
      else {
        response.response = true;
        response.message = "Success";
      }
    }
    catch (const ImpactAcquireException& e) {
      ROS_WARN_STREAM("CameraNode::setFramerate(): "
        "ImpactAcquireException: " << std::endl
        << "serial: " << _device->serial.readS() << std::endl
        << "error code: " << e.getErrorCodeAsString() << std::endl
        << "message: " << e.what());
      response.response = false;
      response.message = e.what();
    }
    return true;
  }

}
