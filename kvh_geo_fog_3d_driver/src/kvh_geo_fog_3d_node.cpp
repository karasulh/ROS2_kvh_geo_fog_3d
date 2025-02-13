/*********************************************************************
 * Software License Agreement (Apache 2.0)
 * 
 *  Copyright (c) 2019, The MITRE Corporation.
 *  All rights reserved.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     https://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Sections of this project contains content developed by The MITRE Corporation.
 * If this code is used in a deployment or embedded within another project,
 * it is requested that you send an email to opensource@mitre.org in order to
 * let us know where this software is being used.
 *********************************************************************/

/**
 * @file kvh_geo_fog_3d_node.cpp
 * @brief Contains code using the kvh driver and eventually the nodelet
 * @author Trevor Bostic
 *
 * @todo Switch publishers to DiagnosticPublisher, which will let us track frequencies (see http://docs.ros.org/api/diagnostic_updater/html/classdiagnostic__updater_1_1DiagnosedPublisher.html)
 */

// STD
#include "unistd.h"
#include <map>
#include <cmath>

// KVH GEO FOG
#include "kvh_geo_fog_3d_driver.hpp"
#include "kvh_geo_fog_3d_global_vars.hpp"
#include "spatial_packets.h"
#include "kvh_diagnostics_container.hpp"
#include "packet_publisher.hpp"

// ROS
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

// Custom ROS msgs
#include <kvh_geo_fog_3d_msgs/msg/kvh_geo_fog3_d_system_state.hpp>
#include <kvh_geo_fog_3d_msgs/msg/kvh_geo_fog3_d_satellites.hpp>
#include <kvh_geo_fog_3d_msgs/msg/kvh_geo_fog3_d_detail_satellites.hpp>
#include <kvh_geo_fog_3d_msgs/msg/kvh_geo_fog3_d_local_magnetic_field.hpp>
#include <kvh_geo_fog_3d_msgs/msg/kvh_geo_fog3_dutm_position.hpp>
#include <kvh_geo_fog_3d_msgs/msg/kvh_geo_fog3_decef_pos.hpp>
#include <kvh_geo_fog_3d_msgs/msg/kvh_geo_fog3_d_north_seeking_init_status.hpp>
#include <kvh_geo_fog_3d_msgs/msg/kvh_geo_fog3_d_odometer_state.hpp>
#include <kvh_geo_fog_3d_msgs/msg/kvh_geo_fog3_d_raw_gnss.hpp>
#include <kvh_geo_fog_3d_msgs/msg/kvh_geo_fog3_d_raw_sensors.hpp>

// Standard ROS msgs
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

// // Bounds on [-pi, pi)
// inline double BoundFromNegPiToPi(const double &_value)
// {
//   double num = std::fmod(_value, (2*M_PI));
//   if (num > M_PI)
//   {
//     num = num - (2*M_PI);
//   }
//   return num;
// } //end: BoundFromNegPiToPi(double* _value)

// inline double BoundFromNegPiToPi(const float &_value)
// {
//   double num = std::fmod(_value, (2*M_PI));
//   if (num > M_PI)
//   {
//     num = num - (2*M_PI);
//   }
//   return num;
// } //end: BoundFromNegPiToPi(const float& _value)

// // Bounds on [-pi, pi)
// inline double BoundFromZeroTo2Pi(const double &_value)
// {
//   return std::fmod(_value, (2 * M_PI));
// } //end: BoundFromZeroTo2Pi(double* _value)

// inline double BoundFromZeroTo2Pi(const float &_value)
// {
//   return std::fmod(_value, (2 * M_PI));
// } //end: BoundFromZeroTo2Pi(const float& _value)

void SetupUpdater(diagnostic_updater::Updater *_diagnostics, mitre::KVH::DiagnosticsContainer *_diagContainer)
{
  _diagnostics->setHardwareID("KVH GEO FOG 3D"); ///< @todo This should probably contain the serial number of the unit, but we only get that after a message read
  /**
   * @todo Add a diagnostics expected packet frequency for important packets and verify
   */
  _diagnostics->add("KVH System", _diagContainer, &mitre::KVH::DiagnosticsContainer::UpdateSystemStatus);
  _diagnostics->add("KVH Filters", _diagContainer, &mitre::KVH::DiagnosticsContainer::UpdateFilterStatus);

}

int GetInitOptions(rclcpp::Node::SharedPtr &_node, kvh::KvhInitOptions &_initOptions)
{

  // Check if the port has been set on the ros param server
  _node->get_parameter("port", _initOptions.port);
  _node->get_parameter("baud", _initOptions.baudRate);
  _node->get_parameter("debug", _initOptions.debugOn);

  int filterVehicleType;
  if (_node->get_parameter("filterVehicleType", filterVehicleType))
  {
    // node.getParam doesn't have an overload for uint8_t
    _initOptions.filterVehicleType = filterVehicleType;
  }

  _node->get_parameter("atmosphericAltitudeEnabled", _initOptions.atmosphericAltitudeEnabled);
  _node->get_parameter("velocityHeadingEnabled", _initOptions.velocityHeadingEnabled);
  _node->get_parameter("reversingDetectionEnabled", _initOptions.reversingDetectionEnabled);
  _node->get_parameter("motionAnalysisEnabled", _initOptions.motionAnalysisEnabled);
  _node->get_parameter("odomPulseToMeters", _initOptions.odomPulseToMeters);
  _node->get_parameter("trackWidth", _initOptions.trackWidth);
  _node->get_parameter("odometerVelocityCovariance", _initOptions.odometerVelocityCovariance);
  _node->get_parameter("encoderOnLeft", _initOptions.encoderOnLeft);

  RCLCPP_INFO_STREAM(_node->get_logger(),"Port: " << _initOptions.port);
  RCLCPP_INFO_STREAM(_node->get_logger(),"Baud: " << _initOptions.baudRate);
  RCLCPP_INFO_STREAM(_node->get_logger(),"Debug: " << _initOptions.debugOn);
  RCLCPP_INFO_STREAM(_node->get_logger(),"Filter Vehicle Type: " << (int)_initOptions.filterVehicleType);
  RCLCPP_INFO_STREAM(_node->get_logger(),"Atmospheric Altitude Enabled: " << _initOptions.atmosphericAltitudeEnabled);
  RCLCPP_INFO_STREAM(_node->get_logger(),"Velocity Heading Enabled: " << _initOptions.velocityHeadingEnabled);
  RCLCPP_INFO_STREAM(_node->get_logger(),"Reversing Detection Enabled: " << _initOptions.reversingDetectionEnabled);
  RCLCPP_INFO_STREAM(_node->get_logger(),"Motion Analysis Enabled: " << _initOptions.motionAnalysisEnabled);
  RCLCPP_INFO_STREAM(_node->get_logger(),"Odometer Pulses to Meters: " << _initOptions.odomPulseToMeters);
  RCLCPP_INFO_STREAM(_node->get_logger(),"Vehicle track width: " << _initOptions.trackWidth);
  RCLCPP_INFO_STREAM(_node->get_logger(),"Odometer velocity covariance: " << _initOptions.odometerVelocityCovariance);
  RCLCPP_INFO_STREAM(_node->get_logger(),"Encoder on left: " << _initOptions.encoderOnLeft);

  return 0;
}


int main(int argc, char **argv)
{

  rclcpp::init(argc,argv);
  auto node = rclcpp::Node::make_shared("kvh_geo_fog_3d_driver_node");
  rclcpp::Rate rate(50); // 50hz by default, may eventually make settable parameter


  diagnostic_updater::Updater diagnostics(node);
  mitre::KVH::DiagnosticsContainer diagContainer;
  SetupUpdater(&diagnostics, &diagContainer);

  // Custom msg publishers
  std::map<packet_id_e, rclcpp::PublisherBase::SharedPtr> kvhPubMap{
      {packet_id_system_state, node->create_publisher<kvh_geo_fog_3d_msgs::msg::KvhGeoFog3DSystemState>("kvh_system_state", 1)},
      {packet_id_satellites, node->create_publisher<kvh_geo_fog_3d_msgs::msg::KvhGeoFog3DSatellites>("kvh_satellites", 1)},
      {packet_id_satellites_detailed, node->create_publisher<kvh_geo_fog_3d_msgs::msg::KvhGeoFog3DDetailSatellites>("kvh_detailed_satellites", 1)},
      {packet_id_local_magnetics, node->create_publisher<kvh_geo_fog_3d_msgs::msg::KvhGeoFog3DLocalMagneticField>("kvh_local_magnetics", 1)},
      {packet_id_utm_position, node->create_publisher<kvh_geo_fog_3d_msgs::msg::KvhGeoFog3DUTMPosition>("kvh_utm_position", 1)},
      {packet_id_ecef_position, node->create_publisher<kvh_geo_fog_3d_msgs::msg::KvhGeoFog3DECEFPos>("kvh_ecef_pos", 1)},
      {packet_id_north_seeking_status, node->create_publisher<kvh_geo_fog_3d_msgs::msg::KvhGeoFog3DNorthSeekingInitStatus>("kvh_north_seeking_status", 1)},
      {packet_id_odometer_state, node->create_publisher<kvh_geo_fog_3d_msgs::msg::KvhGeoFog3DOdometerState>("kvh_odometer_state", 1)},
      {packet_id_raw_sensors, node->create_publisher<kvh_geo_fog_3d_msgs::msg::KvhGeoFog3DRawSensors>("kvh_raw_sensors", 1)},
      {packet_id_raw_gnss, node->create_publisher<kvh_geo_fog_3d_msgs::msg::KvhGeoFog3DRawGNSS>("kvh_raw_gnss", 1)}
  };


  // Publishers for standard ros messages
  
  auto imuRawPub = static_cast<rclcpp::PublisherBase::SharedPtr>(node->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw_frd", 1)); //rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuRawPub
  auto imuRawFLUPub = static_cast<rclcpp::PublisherBase::SharedPtr>(node->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw_flu", 1));
  auto imuNEDPub = static_cast<rclcpp::PublisherBase::SharedPtr>(node->create_publisher<sensor_msgs::msg::Imu>("imu/data_ned", 1));
  auto imuENUPub = static_cast<rclcpp::PublisherBase::SharedPtr>(node->create_publisher<sensor_msgs::msg::Imu>("imu/data_enu", 1));
  auto imuRpyNEDPub = static_cast<rclcpp::PublisherBase::SharedPtr>(node->create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/rpy_ned", 1));
  auto imuRpyNEDDegPub = static_cast<rclcpp::PublisherBase::SharedPtr>(node->create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/rpy_ned_deg", 1));
  auto imuRpyENUPub = static_cast<rclcpp::PublisherBase::SharedPtr>(node->create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/rpy_enu", 1));
  auto imuRpyENUDegPub = static_cast<rclcpp::PublisherBase::SharedPtr>(node->create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/rpy_enu_deg", 1));
  auto navSatFixPub = static_cast<rclcpp::PublisherBase::SharedPtr>(node->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 1));
  auto rawNavSatFixPub = static_cast<rclcpp::PublisherBase::SharedPtr>(node->create_publisher<sensor_msgs::msg::NavSatFix>("gps/raw_fix", 1));
  auto magFieldPub = static_cast<rclcpp::PublisherBase::SharedPtr>(node->create_publisher<sensor_msgs::msg::MagneticField>("mag", 1));
  auto odomPubNED = static_cast<rclcpp::PublisherBase::SharedPtr>(node->create_publisher<nav_msgs::msg::Odometry>("gps/utm_ned", 1));
  auto odomPubENU = static_cast<rclcpp::PublisherBase::SharedPtr>(node->create_publisher<nav_msgs::msg::Odometry>("gps/utm_enu", 1));
  auto odomStatePub = static_cast<rclcpp::PublisherBase::SharedPtr>(node->create_publisher<nav_msgs::msg::Odometry>("odom/wheel_encoder", 1));
  auto odomSpeedPub = static_cast<rclcpp::PublisherBase::SharedPtr>(node->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("odom/encoder_vehicle_velocity", 1));
  auto rawSensorImuPub = static_cast<rclcpp::PublisherBase::SharedPtr>(node->create_publisher<sensor_msgs::msg::Imu>("imu/raw_sensor_frd", 1));
  auto rawSensorImuFluPub = static_cast<rclcpp::PublisherBase::SharedPtr>(node->create_publisher<sensor_msgs::msg::Imu>("imu/raw_sensor_flu", 1));
  auto velNEDTwistPub = static_cast<rclcpp::PublisherBase::SharedPtr>(node->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("gps/vel_ned", 1));
  auto velENUTwistPub = static_cast<rclcpp::PublisherBase::SharedPtr>(node->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("gps/vel_enu", 1));
  auto velBodyTwistFLUPub = static_cast<rclcpp::PublisherBase::SharedPtr>(node->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("imu/vel_flu", 1));
  auto velBodyTwistFRDPub = static_cast<rclcpp::PublisherBase::SharedPtr>(node->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("imu/vel_frd", 1));


  //////////////////////////
  // KVH Setup
  //////////////////////////

  // To get packets from the driver, we first create a vector
  // that holds a pair containing the packet id and the desired frequency for it to be published
  // See documentation for all id's.
  typedef std::pair<packet_id_e, int> freqPair;

  kvh::KvhPacketRequest packetRequest{
      freqPair(packet_id_euler_orientation_standard_deviation, 50),
      freqPair(packet_id_system_state, 50),
      freqPair(packet_id_satellites, 10),
      freqPair(packet_id_satellites_detailed, 1),
      freqPair(packet_id_local_magnetics, 50),
      freqPair(packet_id_utm_position, 50),
      freqPair(packet_id_ecef_position, 50),
      freqPair(packet_id_north_seeking_status, 50),
      freqPair(packet_id_odometer_state, 50),
      freqPair(packet_id_raw_sensors, 50),
      freqPair(packet_id_raw_gnss, 50),
      freqPair(packet_id_body_velocity, 50),
      freqPair(packet_id_velocity_standard_deviation, 50),
  };

  kvh::Driver kvhDriver;
  kvh::KvhInitOptions initOptions;

  if (GetInitOptions(node, initOptions) < 0)
  {
    RCLCPP_ERROR_STREAM(node->get_logger(),"Unable to get init options. Exiting.");
    exit(1);
  }

  int errorCode;
  if ((errorCode = kvhDriver.Init(initOptions.port, packetRequest, initOptions)) < 0)
  {
    RCLCPP_ERROR_STREAM(node->get_logger(),"Unable to initialize driver. Error Code "<< errorCode);
    exit(1);
  };

  // Request odom configuration packet to get the pulse length
  // \todo Refactor! We have to do the below due to some poor design decisions
  // on my part. The odometer configuration packet cannot be added to the
  // packet request above since adding it to there will cause it to be added
  // to the packet periods packet, which in turn throws an error.
  kvhDriver.AddPacket(packet_id_odometer_configuration);
  kvhDriver.RequestPacket(packet_id_odometer_configuration);

  // Declare these for reuse
  system_state_packet_t systemStatePacket;
  satellites_packet_t satellitesPacket;
  detailed_satellites_packet_t detailSatellitesPacket;
  local_magnetics_packet_t localMagPacket;
  kvh::utm_fix utmPosPacket;
  ecef_position_packet_t ecefPosPacket;
  north_seeking_status_packet_t northSeekingStatPacket;
  euler_orientation_standard_deviation_packet_t eulStdDevPack;
  odometer_state_packet_t odomStatePacket;
  raw_sensors_packet_t rawSensorsPacket;
  raw_gnss_packet_t rawGnssPacket;
  body_velocity_packet_t bodyVelocityPacket;
  velocity_standard_deviation_packet_t velocityStdDevPack;
  odometer_configuration_packet_t odomConfigPacket;

  // Default value for pulse to meters in case it is not updated from kvh
  double odomPulseToMeters = initOptions.odomPulseToMeters;

  auto steady_clock = *(node->get_clock());//rclcpp::Clock(); //Todo?: It may be change to another time source.

  while (rclcpp::ok())
  {
    // Collect packet data
    kvhDriver.Once();

    // If packets were updated, record the data, we will use the same instance
    // of the packets for the remainder of this loop

    if (kvhDriver.PacketIsUpdated(packet_id_system_state))
    {
      RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(),steady_clock,1000,"System state packet has been updated.");
      kvhDriver.GetPacket(packet_id_system_state, systemStatePacket);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_satellites))
    {
      RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(),steady_clock,1000,"Satellites packet has been updated.");
      kvhDriver.GetPacket(packet_id_satellites, satellitesPacket);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_satellites_detailed))
    {
      RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(),steady_clock,1000,"Detailed satellites packet has been updated.");
      kvhDriver.GetPacket(packet_id_satellites_detailed, detailSatellitesPacket);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_local_magnetics))
    {
      RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(),steady_clock,1000,"Local Mag packet has been updated.");
      kvhDriver.GetPacket(packet_id_local_magnetics, localMagPacket);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_utm_position))
    {
      RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(),steady_clock,1000,"Utm position packet has been updated.");
      kvhDriver.GetPacket(packet_id_utm_position, utmPosPacket);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_ecef_position))
    {
      RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(),steady_clock,1000,"Ecef packet has been updated.");
      kvhDriver.GetPacket(packet_id_ecef_position, ecefPosPacket);
    }
    if (kvhDriver.PacketIsUpdated(packet_id_north_seeking_status))
    {
      RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(),steady_clock,1000,"North Seeking Status packet has been updated.");
      kvhDriver.GetPacket(packet_id_north_seeking_status, northSeekingStatPacket);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_euler_orientation_standard_deviation))
    {
      RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(),steady_clock,1000,"Euler Std Dev packet has been updated.");
      kvhDriver.GetPacket(packet_id_euler_orientation_standard_deviation, eulStdDevPack);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_odometer_state))
    {
      RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(),steady_clock,1000,"Odom State packet has been updated.");
      kvhDriver.GetPacket(packet_id_odometer_state, odomStatePacket);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_raw_sensors))
    {
      RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(),steady_clock,1000,"Raw Sensors packet has been updated.");
      kvhDriver.GetPacket(packet_id_raw_sensors, rawSensorsPacket);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_raw_gnss))
    {
      RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(),steady_clock,1000,"Raw Gnss packet has been updated.");
      kvhDriver.GetPacket(packet_id_raw_gnss, rawGnssPacket);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_body_velocity))
    {
      RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(),steady_clock,1000,"Body Velocity packet has been updated.");
      kvhDriver.GetPacket(packet_id_body_velocity, bodyVelocityPacket);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_odometer_configuration))
    {
      RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(),steady_clock,1000,"Odometer configuration packet has been updated.");
      kvhDriver.GetPacket(packet_id_odometer_configuration, odomConfigPacket);
    }

    ///////////////////////////////////////////
    // OUTPUT ROS MESSAGES AND DIAGNOSTICS
    ///////////////////////////////////////////

    // SYSTEM STATE PACKET
    if (kvhDriver.PacketIsUpdated(packet_id_system_state))
    {
      // Messages that are dependent ONLY on the system state packet
      PublishSystemState(kvhPubMap[packet_id_system_state], systemStatePacket,steady_clock);
      PublishIMURaw(imuRawPub, systemStatePacket,steady_clock);
      PublishIMURawFLU(imuRawFLUPub, systemStatePacket,steady_clock);
      PublishIMU_RPY_NED(imuRpyNEDPub, systemStatePacket,steady_clock);
      PublishIMU_RPY_NED_DEG(imuRpyNEDDegPub, systemStatePacket,steady_clock);
      PublishIMU_RPY_ENU(imuRpyENUPub, systemStatePacket,steady_clock);
      PublishIMU_RPY_ENU_DEG(imuRpyENUDegPub, systemStatePacket,steady_clock);
      PublishNavSatFix(navSatFixPub, systemStatePacket,steady_clock);

      //Update diagnostics container from this message
      diagContainer.SetSystemStatus(systemStatePacket.system_status.r);
      diagContainer.SetFilterStatus(systemStatePacket.filter_status.r);

      // Messages dependent on system state AND euler std dev
      if (kvhDriver.PacketIsUpdated(packet_id_euler_orientation_standard_deviation))
      {

        // CAREFUL!!! Sometimes this packet will return NANs for some reason
        if (std::isnan(eulStdDevPack.standard_deviation[0]))
        {
          eulStdDevPack.standard_deviation[0] = 0;
          RCLCPP_INFO_STREAM(node->get_logger(),"NAN Found");
        }

        if (std::isnan(eulStdDevPack.standard_deviation[1]))
        {
          eulStdDevPack.standard_deviation[1] = 0;
          RCLCPP_INFO_STREAM(node->get_logger(),"NAN Found");
        }

        if (std::isnan(eulStdDevPack.standard_deviation[2]))
        {
          eulStdDevPack.standard_deviation[2] = 0;
          RCLCPP_INFO_STREAM(node->get_logger(),"NAN Found");
        }

        PublishIMU_NED(imuNEDPub, systemStatePacket, eulStdDevPack,steady_clock);
        PublishIMU_ENU(imuENUPub, systemStatePacket, eulStdDevPack,steady_clock);

        // System state, eul std dev, utm, and body velocity
        if (kvhDriver.PacketIsUpdated(packet_id_utm_position) &&
            kvhDriver.PacketIsUpdated(packet_id_body_velocity))
        {
          PublishOdomNED(odomPubNED, systemStatePacket, utmPosPacket, eulStdDevPack, bodyVelocityPacket,steady_clock);
          PublishOdomENU(odomPubENU, systemStatePacket, utmPosPacket, eulStdDevPack, bodyVelocityPacket,steady_clock);
        }
      }

      // System State AND Raw Gnss
      if (kvhDriver.PacketIsUpdated(packet_id_raw_gnss))
      {
        PublishRawNavSatFix(rawNavSatFixPub, systemStatePacket, rawGnssPacket,steady_clock);
      }

      // System State AND Odometer State
      if (kvhDriver.PacketIsUpdated(packet_id_odometer_state))
      {
        PublishOdomSpeed(odomSpeedPub, systemStatePacket, odomStatePacket, initOptions.trackWidth,
        initOptions.odometerVelocityCovariance, initOptions.encoderOnLeft,steady_clock);
      }

      // System State and Velocity Std Dev
      if (kvhDriver.PacketIsUpdated(packet_id_velocity_standard_deviation))
      {
        PublishVelNEDTwist(velNEDTwistPub, systemStatePacket, velocityStdDevPack,steady_clock);
        PublishVelENUTwist(velENUTwistPub, systemStatePacket, velocityStdDevPack,steady_clock);
      }

      // System state, body velocity AND Velocity Std Dev
      if (kvhDriver.PacketIsUpdated(packet_id_body_velocity) &&
        kvhDriver.PacketIsUpdated(packet_id_velocity_standard_deviation))
        {
          PublishVelBodyTwistFLU(velBodyTwistFLUPub, systemStatePacket, bodyVelocityPacket, velocityStdDevPack,steady_clock);
          PublishVelBodyTwistFRD(velBodyTwistFRDPub, systemStatePacket, bodyVelocityPacket, velocityStdDevPack,steady_clock);
        }
    }

    // SATELLITES PACKET
    if (kvhDriver.PacketIsUpdated(packet_id_satellites))
    {
      RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(),steady_clock,3000,"Satellites packet updated. Publishing...");
      kvhDriver.GetPacket(packet_id_satellites, satellitesPacket);

      PublishSatellites(kvhPubMap[packet_id_satellites], satellitesPacket,steady_clock);
    }

    // SATELLITES DETAILED
    if (kvhDriver.PacketIsUpdated(packet_id_satellites_detailed))
    {
      RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(),steady_clock,3000,"Detailed satellites packet updated. Publishing...");
      kvhDriver.GetPacket(packet_id_satellites_detailed, detailSatellitesPacket);

      PublishSatellitesDetailed(kvhPubMap[packet_id_satellites_detailed], detailSatellitesPacket,steady_clock);
    }

    // LOCAL MAGNETICS PACKET
    if (kvhDriver.PacketIsUpdated(packet_id_local_magnetics))
    {
      RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(),steady_clock,3000,"Local magnetics packet updated. Publishing...");
      kvhDriver.GetPacket(packet_id_local_magnetics, localMagPacket);

      PublishLocalMagnetics(kvhPubMap[packet_id_local_magnetics], localMagPacket,steady_clock);
    }

    // UTM POSITION PACKET
    if (kvhDriver.PacketIsUpdated(packet_id_utm_position))
    {
      RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(),steady_clock,3000,"UTM Position packet updated. Publishing...");
      kvhDriver.GetPacket(packet_id_utm_position, utmPosPacket);

      PublishUtmPosition(kvhPubMap[packet_id_utm_position], utmPosPacket,steady_clock);
    }

    // ECEF POSITION PACKET
    if (kvhDriver.PacketIsUpdated(packet_id_ecef_position))
    {
      RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(),steady_clock,3000,"ECEF position packet updated. Publishing...");
      kvhDriver.GetPacket(packet_id_ecef_position, ecefPosPacket);

      PublishEcefPosition(kvhPubMap[packet_id_ecef_position], ecefPosPacket,steady_clock);
    }

    // NORTH SEEKING STATUS PACKET
    if (kvhDriver.PacketIsUpdated(packet_id_north_seeking_status))
    {
      RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(),steady_clock,3000,"North seeking status packet updated. Publishing...");
      kvhDriver.GetPacket(packet_id_north_seeking_status, northSeekingStatPacket);

      PublishNorthSeekingStatus(kvhPubMap[packet_id_north_seeking_status], northSeekingStatPacket,steady_clock);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_odometer_state))
    {
      RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(),steady_clock,3000,"Odometer state updated. Publishing...");
      kvhDriver.GetPacket(packet_id_odometer_state, odomStatePacket);

      PublishKvhOdometerState(kvhPubMap[packet_id_odometer_state], odomStatePacket,steady_clock);
      PublishOdomState(odomStatePub, odomStatePacket, odomPulseToMeters,steady_clock);
    }

    if (kvhDriver.PacketIsUpdated(packet_id_raw_sensors))
    {
      RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(),steady_clock,3000,"Raw sensors packet updated. Publishing...");
      kvhDriver.GetPacket(packet_id_raw_sensors, rawSensorsPacket);

      PublishRawSensors(kvhPubMap[packet_id_raw_sensors], rawSensorsPacket,steady_clock);
      PublishMagField(magFieldPub, rawSensorsPacket,steady_clock);
      PublishIMUSensorRaw(rawSensorImuPub, rawSensorsPacket,steady_clock);
      PublishIMUSensorRawFLU(rawSensorImuFluPub, rawSensorsPacket,steady_clock);
    }

    /**
     * @attention The raw gnss has an additional field called floating
     * ambiguity heading. It is not implemented in their api. If we wish to
     * implement this, we would need to make an extension similar to what we
     * did for the UTM packet.
     */
    if (kvhDriver.PacketIsUpdated(packet_id_raw_gnss))
    {
      RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(),steady_clock,3000,"Raw GNSS packet updated. Publishing...");
      kvhDriver.GetPacket(packet_id_raw_gnss, rawGnssPacket);

      PublishRawGnss(kvhPubMap[packet_id_raw_gnss], rawGnssPacket,steady_clock);
    }

    // Get the kvh setting for pulse length to use below in the distance travelled calculation
    if (kvhDriver.PacketIsUpdated(packet_id_odometer_configuration))
    {
      RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(),steady_clock,3000,"Obtaining pulse length from odometer config.");
      kvhDriver.GetPacket(packet_id_odometer_configuration, odomConfigPacket);
      // Assume if it is not 0 then it has been calibrated
      if (odomConfigPacket.pulse_length != 0)
      {
        odomPulseToMeters = odomConfigPacket.pulse_length;
      }
    }

    // Set that we have read the latest versions of all packets. There is a small possibility we miss one packet
    // between using it above and setting it here.
    kvhDriver.SetPacketUpdated(packet_id_system_state, false);
    kvhDriver.SetPacketUpdated(packet_id_satellites, false);
    kvhDriver.SetPacketUpdated(packet_id_satellites_detailed, false);
    kvhDriver.SetPacketUpdated(packet_id_utm_position, false);
    kvhDriver.SetPacketUpdated(packet_id_ecef_position, false);
    kvhDriver.SetPacketUpdated(packet_id_north_seeking_status, false);
    kvhDriver.SetPacketUpdated(packet_id_local_magnetics, false);
    kvhDriver.SetPacketUpdated(packet_id_euler_orientation_standard_deviation, false);
    kvhDriver.SetPacketUpdated(packet_id_odometer_state, false);
    kvhDriver.SetPacketUpdated(packet_id_raw_gnss, false);
    kvhDriver.SetPacketUpdated(packet_id_raw_sensors, false);
    kvhDriver.SetPacketUpdated(packet_id_body_velocity, false);
    kvhDriver.SetPacketUpdated(packet_id_velocity_standard_deviation, false);
    kvhDriver.SetPacketUpdated(packet_id_odometer_configuration, false);

    //diagnostics.update();//I think in ros2, no need to update for diagnostics.

    rclcpp::spin_some(node);
    rate.sleep();
    RCLCPP_DEBUG_STREAM_THROTTLE(node->get_logger(),steady_clock,3000,"----------------------------------------");

  }

  diagnostics.broadcast(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Shutting down the KVH driver");
  kvhDriver.Cleanup();

  rclcpp::shutdown();
  return 0;
}

