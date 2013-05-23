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
    ROS_INFO_STREAM("CameraManagerNode::CameraManagerNode(): " << std::endl
      << "library version: "
      << _deviceManager->getVersionAsString(lqDeviceManager));
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
          if (it->second->isMaster())
            ROS_WARN_STREAM("CameraManagerNode::updateDeviceList(): "
              "removing master camera");
          ROS_INFO_STREAM("CameraManagerNode::updateDeviceList(): " << std::endl
            << "removing device: " << it->second->getSerial());
          it->second->interrupt();
          _cameraNodes.erase(it->first);
        }
      }
      // updating camera list and launching new one
      _deviceManager->updateDeviceList();
      const size_t numDevices = _deviceManager->deviceCount();
      ROS_INFO_STREAM("CameraManagerNode::updateDeviceList(): " << std::endl
        << "number of active devices: " << _cameraNodes.size());
      for (size_t i = 0; i < numDevices; ++i) {
        Device* device = (*_deviceManager)[i];
        if (device->state.read() == dsPresent &&
            !_cameraNodes.count(device->serial.readS())) {
          bool isMaster = false;
          if (device->serial.readS() == _masterCameraSerial)
            isMaster = true;
          ROS_INFO_STREAM("CameraManagerNode::updateDeviceList(): "
            "adding device: " << std::endl
            << "DeviceClass: " << device->deviceClass.readS() << std::endl
            << "Family: " << device->family.read() << std::endl
            << "Product: " << device->product.read() << std::endl
            << "Serial: " << device->serial.read() << std::endl
            << "State: " << device->state.readS() << std::endl
            << "DeviceID: " << device->deviceID.read() << std::endl
            << "DeviceVersion: " << device->deviceVersion.read() << std::endl
            << "FirmwareVersion: "
            << device->firmwareVersion.getTranslationDictString() << std::endl
            << "LoadSettings: "
            << device->loadSettings.getTranslationDictString() << std::endl
            << "InterfaceLayout: "
            << device->interfaceLayout.readS());
          _cameraNodes[device->serial.readS()] = std::shared_ptr<CameraNode>(
            new CameraNode(_nodeHandle, device, isMaster));
          _cameraNodes[device->serial.readS()]->start();
        }
      }
    }
    catch (ImpactAcquireException& e) {
      ROS_WARN_STREAM("CameraManagerNode::updateDeviceList(): "
        "ImpactAcquireException: " << std::endl
        << "error code: " << e.getErrorCodeAsString() << std::endl
        << "message: " << e.what());
      ROS_WARN_STREAM("Retrying in " << _updateDeviceListTime << " [s]");
    }
    catch (SystemException& e) {
      ROS_WARN_STREAM("CameraManagerNode::updateDeviceList(): "
        "SystemException: " << std::endl
        << "message: " << e.what());
      ROS_WARN_STREAM("Retrying in " << _updateDeviceListTime << " [s]");
    }
    catch (InvalidOperationException& e) {
      ROS_WARN_STREAM("CameraManagerNode::updateDeviceList(): "
        "InvalidOperationException: " << std::endl
        << "message: " << e.what());
      ROS_WARN_STREAM("Retrying in " << _updateDeviceListTime << " [s]");
    }
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
  }

}
