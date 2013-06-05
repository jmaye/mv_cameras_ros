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

#include "sensor/CameraManagerNode.h"

#include <sstream>
#include <vector>

#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

#include "sensor/CameraNode.h"
#include "exceptions/SystemException.h"
#include "exceptions/InvalidOperationException.h"

using namespace mvIMPACT::acquire;

namespace mv {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  CameraManagerNode::CameraManagerNode(const ros::NodeHandle& nh) :
      _nodeHandle(nh) {
    getParameters();
    _deviceManager.reset(new DeviceManager());
    _driverVersion = _deviceManager->getVersionAsString(lqDeviceManager);
    _updater.setHardwareID("mv_cameras_manager");
    _cmFreq.reset(new diagnostic_updater::FrequencyStatus(
      diagnostic_updater::FrequencyStatusParam(
      &_discoverMinFreq, &_discoverMaxFreq, 0.1, 60)));
    _updater.add("Frequency", _cmFreq.get(),
      &diagnostic_updater::FrequencyStatus::run);
    _updater.add("Camera manager", this,
      &CameraManagerNode::diagnoseCameraManager);
    _setExposuresService = _nodeHandle.advertiseService("set_exposures",
      &CameraManagerNode::setExposures, this);
    _setGainsService = _nodeHandle.advertiseService("set_gains",
      &CameraManagerNode::setGains, this);
    _setFrameratesService = _nodeHandle.advertiseService("set_framerates",
      &CameraManagerNode::setFramerates, this);
    _updater.force_update();
  }

  CameraManagerNode::~CameraManagerNode() {
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void CameraManagerNode::updateDeviceList(const ros::TimerEvent& event) {
    try {
      // looking over the existing camera list and removing unplugged one
      for (auto it = _cameraNodes.begin(); it != _cameraNodes.end(); ++it) {
        if (it->second->getState().read() == dsAbsent) {
          it->second->interrupt();
          _cameraNodes.erase(it->first);
        }
      }
      // updating camera list and launching new one
      _deviceManager->updateDeviceList();
      const size_t numDevices = _deviceManager->deviceCount();
      for (size_t i = 0; i < numDevices; ++i) {
        Device* device = (*_deviceManager)[i];
        if (device->state.read() == dsPresent &&
            !_cameraNodes.count(device->serial.readS())) {
          bool isMaster = false;
          if (device->serial.readS() == _masterCameraSerial)
            isMaster = true;
          _cameraNodes[device->serial.readS()] = std::shared_ptr<CameraNode>(
            new CameraNode(_nodeHandle, device, isMaster));
          _cameraNodes[device->serial.readS()]->start();
        }
      }
    }
    catch (const ImpactAcquireException& e) {
      ROS_WARN_STREAM("CameraManagerNode::updateDeviceList(): "
        "ImpactAcquireException: " << std::endl
        << "error code: " << e.getErrorCodeAsString() << std::endl
        << "message: " << e.what());
      ROS_WARN_STREAM("Retrying in " << _updateDeviceListTime << " [s]");
    }
    catch (const SystemException& e) {
      ROS_WARN_STREAM("CameraManagerNode::updateDeviceList(): "
        "SystemException: " << std::endl
        << "message: " << e.what());
      ROS_WARN_STREAM("Retrying in " << _updateDeviceListTime << " [s]");
    }
    catch (const InvalidOperationException& e) {
      ROS_WARN_STREAM("CameraManagerNode::updateDeviceList(): "
        "InvalidOperationException: " << std::endl
        << "message: " << e.what());
      ROS_WARN_STREAM("Retrying in " << _updateDeviceListTime << " [s]");
    }
    catch (...) {
      ROS_WARN_STREAM("CameraManagerNode::updateDeviceList(): "
        "Unknown exception");
      ROS_WARN_STREAM("Retrying in " << _updateDeviceListTime << " [s]");
    }
    _cmFreq->tick();
    _updater.update();
  }

  int CameraManagerNode::spin() {
    ros::Timer updateDeviceListTimer = _nodeHandle.createTimer(
      ros::Duration(_updateDeviceListTime),
      &CameraManagerNode::updateDeviceList, this);
    updateDeviceList(ros::TimerEvent());
    ros::spin();
  }

  void CameraManagerNode::getParameters() {
    _nodeHandle.param<double>("device_manager/update_device_list_time",
      _updateDeviceListTime, 10.0);
    _nodeHandle.param<std::string>("device_manager/master_camera_serial",
      _masterCameraSerial, "GX002409");
    _nodeHandle.param<double>("device_manager/diagnostics/discover_min_freq",
      _discoverMinFreq, 0.1);
    _nodeHandle.param<double>("device_manager/diagnostics/discover_max_freq",
      _discoverMaxFreq, 0.2);
  }

  void CameraManagerNode::diagnoseCameraManager(
      diagnostic_updater::DiagnosticStatusWrapper& status) {
    if (_cameraNodes.count(_masterCameraSerial))
      status.summary(diagnostic_msgs::DiagnosticStatus::OK,
        "Master camera running.");
    else
      status.summary(diagnostic_msgs::DiagnosticStatus::WARN,
        "Master camera not running.");
    status.add("Driver version", _driverVersion);
    status.add("Master camera", _masterCameraSerial);
    status.add("Connected cameras", _cameraNodes.size());
    size_t idx = 0;
    for (auto it = _cameraNodes.cbegin(); it != _cameraNodes.cend(); ++it) {
      std::stringstream keySs;
      keySs << "Camera " << idx++;
      status.add(keySs.str(), it->first);
    }
  }

  bool CameraManagerNode::setExposures(mv_cameras::SetExposure::Request&
      request, mv_cameras::SetExposure::Response& response) {
    const double exposureTime = request.exposure;
    std::vector<std::string> camSuccess;
    std::vector<std::string> camFailure;
    std::unordered_map<std::string, std::string> camErrors;
    for (auto it = _cameraNodes.begin(); it != _cameraNodes.end(); ++it) {
      if (it->second->getState().read() == dsPresent) {
        ros::ServiceClient client =
          _nodeHandle.serviceClient<mv_cameras::SetExposure>(
          "/mv_cameras_manager/" + it->first + "/set_exposure");
        mv_cameras::SetExposure srv;
        srv.request.exposure = exposureTime;
        if (client.call(srv)) {
          if (!srv.response.response) {
            camFailure.push_back(it->first);
            camErrors[it->first] = srv.response.message;
          }
          else {
            camSuccess.push_back(it->first);
          }
        }
        else {
          camFailure.push_back(it->first);
          camErrors[it->first] = srv.response.message;
        }
      }
    }
    std::stringstream camSuccessSs;
    for (auto it = camSuccess.cbegin(); it != camSuccess.cend(); ++it)
      camSuccessSs << *it << " ";
    std::stringstream camFailureSs;
    for (auto it = camFailure.cbegin(); it != camFailure.cend(); ++it)
      camFailureSs << *it << ": " << camErrors[*it] << std::endl;
    if (camFailure.size() == 0 && camSuccess.size() == 0) {
      response.response = false;
      response.message = "No cameras connected";
    }
    else if (camFailure.size() == 0) {
      response.response = true;
      response.message = "Exposure successfully set on: " + camSuccessSs.str();
    }
    else if (camSuccess.size() > 0) {
      response.response = false;
      std::stringstream messageSs;
      messageSs << "Exposure successfully set on: " + camSuccessSs.str()
        << std::endl
        << "Exposure failed on: " << std::endl << camFailureSs.str();
      response.message = messageSs.str();
    }
    else {
      response.response = false;
      std::stringstream messageSs;
      messageSs << "Exposure failed on: " << std::endl << camFailureSs.str();
      response.message = messageSs.str();
    }
    return true;
  }

  bool CameraManagerNode::setGains(mv_cameras::SetGain::Request& request,
      mv_cameras::SetGain::Response& response) {
    const double gain = request.gain;
    std::vector<std::string> camSuccess;
    std::vector<std::string> camFailure;
    std::unordered_map<std::string, std::string> camErrors;
    for (auto it = _cameraNodes.begin(); it != _cameraNodes.end(); ++it) {
      if (it->second->getState().read() == dsPresent) {
        ros::ServiceClient client =
          _nodeHandle.serviceClient<mv_cameras::SetGain>(
          "/mv_cameras_manager/" + it->first + "/set_gain");
        mv_cameras::SetGain srv;
        srv.request.gain = gain;
        if (client.call(srv)) {
          if (!srv.response.response) {
            camFailure.push_back(it->first);
            camErrors[it->first] = srv.response.message;
          }
          else {
            camSuccess.push_back(it->first);
          }
        }
        else {
          camFailure.push_back(it->first);
          camErrors[it->first] = srv.response.message;
        }
      }
    }
    std::stringstream camSuccessSs;
    for (auto it = camSuccess.cbegin(); it != camSuccess.cend(); ++it)
      camSuccessSs << *it << " ";
    std::stringstream camFailureSs;
    for (auto it = camFailure.cbegin(); it != camFailure.cend(); ++it)
      camFailureSs << *it << ": " << camErrors[*it] << std::endl;
    if (camFailure.size() == 0 && camSuccess.size() == 0) {
      response.response = false;
      response.message = "No cameras connected";
    }
    else if (camFailure.size() == 0) {
      response.response = true;
      response.message = "Gain successfully set on: " + camSuccessSs.str();
    }
    else if (camSuccess.size() > 0) {
      response.response = false;
      std::stringstream messageSs;
      messageSs << "Gain successfully set on: " + camSuccessSs.str()
        << std::endl
        << "Gain failed on: " << std::endl << camFailureSs.str();
      response.message = messageSs.str();
    }
    else {
      response.response = false;
      std::stringstream messageSs;
      messageSs << "Gain failed on: " << std::endl << camFailureSs.str();
      response.message = messageSs.str();
    }
    return true;
  }

  bool CameraManagerNode::setFramerates(mv_cameras::SetFramerate::Request&
      request, mv_cameras::SetFramerate::Response& response) {
    const double framerate = request.framerate;
    std::vector<std::string> camSuccess;
    std::vector<std::string> camFailure;
    std::unordered_map<std::string, std::string> camErrors;
    for (auto it = _cameraNodes.begin(); it != _cameraNodes.end(); ++it) {
      if (it->second->getState().read() == dsPresent) {
        ros::ServiceClient client =
          _nodeHandle.serviceClient<mv_cameras::SetFramerate>(
          "/mv_cameras_manager/" + it->first + "/set_framerate");
        mv_cameras::SetFramerate srv;
        srv.request.framerate = framerate;
        if (client.call(srv)) {
          if (!srv.response.response) {
            camFailure.push_back(it->first);
            camErrors[it->first] = srv.response.message;
          }
          else {
            camSuccess.push_back(it->first);
          }
        }
        else {
          camFailure.push_back(it->first);
          camErrors[it->first] = srv.response.message;
        }
      }
    }
    std::stringstream camSuccessSs;
    for (auto it = camSuccess.cbegin(); it != camSuccess.cend(); ++it)
      camSuccessSs << *it << " ";
    std::stringstream camFailureSs;
    for (auto it = camFailure.cbegin(); it != camFailure.cend(); ++it)
      camFailureSs << *it << ": " << camErrors[*it] << std::endl;
    if (camFailure.size() == 0 && camSuccess.size() == 0) {
      response.response = false;
      response.message = "No cameras connected";
    }
    else if (camFailure.size() == 0) {
      response.response = true;
      response.message = "Framerate successfully set on: " + camSuccessSs.str();
    }
    else if (camSuccess.size() > 0) {
      response.response = false;
      std::stringstream messageSs;
      messageSs << "Framerate successfully set on: " + camSuccessSs.str()
        << std::endl
        << "Framerate failed on: " << std::endl << camFailureSs.str();
      response.message = messageSs.str();
    }
    else {
      response.response = false;
      std::stringstream messageSs;
      messageSs << "Framerate failed on: " << std::endl << camFailureSs.str();
      response.message = messageSs.str();
    }
    return true;
  }

}
