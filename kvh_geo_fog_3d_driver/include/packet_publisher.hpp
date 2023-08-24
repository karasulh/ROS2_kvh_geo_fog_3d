#pragma once

// KVH
#include "an_packet_protocol.h"
#include "spatial_packets.h"
#include "kvh_geo_fog_3d_global_vars.hpp"

// CUSTOM ROS MESSAGES
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

// ROS
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

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


// Bound rotations
double BoundFromNegPiToPi(const double);
double BoundFromNegPiToPi(const float);
double BoundFromZeroTo2Pi(const double);
double BoundFromZeroTo2Pi(const float);

// RPY to quaternion conversion
tf2::Quaternion quatFromRPY(double roll, double pitch, double yaw);

// Custom ROS Message
void PublishSystemState(rclcpp::PublisherBase::SharedPtr&, system_state_packet_t,rclcpp::Clock&);
void PublishSatellites(rclcpp::PublisherBase::SharedPtr&, satellites_packet_t,rclcpp::Clock&);
void PublishSatellitesDetailed(rclcpp::PublisherBase::SharedPtr&, detailed_satellites_packet_t,rclcpp::Clock&);
void PublishLocalMagnetics(rclcpp::PublisherBase::SharedPtr&, local_magnetics_packet_t,rclcpp::Clock&);
void PublishUtmPosition(rclcpp::PublisherBase::SharedPtr&, utm_position_packet_t,rclcpp::Clock&);
void PublishEcefPosition(rclcpp::PublisherBase::SharedPtr&, ecef_position_packet_t,rclcpp::Clock&);
void PublishNorthSeekingStatus(rclcpp::PublisherBase::SharedPtr&, north_seeking_status_packet_t,rclcpp::Clock&);
void PublishKvhOdometerState(rclcpp::PublisherBase::SharedPtr&, odometer_state_packet_t,rclcpp::Clock&);
void PublishRawSensors(rclcpp::PublisherBase::SharedPtr&, raw_sensors_packet_t,rclcpp::Clock&);
void PublishRawGnss(rclcpp::PublisherBase::SharedPtr&, raw_gnss_packet_t,rclcpp::Clock&);

// Standard ROS Messages
void PublishIMURaw(rclcpp::PublisherBase::SharedPtr&, system_state_packet_t,rclcpp::Clock&);
void PublishIMURawFLU(rclcpp::PublisherBase::SharedPtr&, system_state_packet_t,rclcpp::Clock&);
void PublishIMU_NED(rclcpp::PublisherBase::SharedPtr&, system_state_packet_t, euler_orientation_standard_deviation_packet_t,rclcpp::Clock&);
void PublishIMU_ENU(rclcpp::PublisherBase::SharedPtr&, system_state_packet_t, euler_orientation_standard_deviation_packet_t,rclcpp::Clock&);
void PublishIMU_RPY_NED(rclcpp::PublisherBase::SharedPtr&, system_state_packet_t,rclcpp::Clock&);
void PublishIMU_RPY_NED_DEG(rclcpp::PublisherBase::SharedPtr&, system_state_packet_t,rclcpp::Clock&);
void PublishIMU_RPY_ENU(rclcpp::PublisherBase::SharedPtr&, system_state_packet_t,rclcpp::Clock&);
void PublishIMU_RPY_ENU_DEG(rclcpp::PublisherBase::SharedPtr&, system_state_packet_t,rclcpp::Clock&);
void PublishNavSatFix(rclcpp::PublisherBase::SharedPtr&, system_state_packet_t,rclcpp::Clock&);
void PublishRawNavSatFix(rclcpp::PublisherBase::SharedPtr&, system_state_packet_t, raw_gnss_packet_t,rclcpp::Clock&);
void PublishMagField(rclcpp::PublisherBase::SharedPtr&, raw_sensors_packet_t,rclcpp::Clock&);
void PublishOdomNED(rclcpp::PublisherBase::SharedPtr&, system_state_packet_t, utm_position_packet_t, 
    euler_orientation_standard_deviation_packet_t, body_velocity_packet_t,rclcpp::Clock&);
void PublishOdomENU(rclcpp::PublisherBase::SharedPtr&, system_state_packet_t , utm_position_packet_t,
                    euler_orientation_standard_deviation_packet_t, body_velocity_packet_t,rclcpp::Clock&);
void PublishOdomState(rclcpp::PublisherBase::SharedPtr&, odometer_state_packet_t, double,rclcpp::Clock&);
void PublishOdomSpeed(rclcpp::PublisherBase::SharedPtr&, system_state_packet_t, odometer_state_packet_t, double, double, bool,rclcpp::Clock&);
void PublishIMUSensorRaw(rclcpp::PublisherBase::SharedPtr&, raw_sensors_packet_t,rclcpp::Clock&);
void PublishIMUSensorRawFLU(rclcpp::PublisherBase::SharedPtr&, raw_sensors_packet_t,rclcpp::Clock&);
void PublishVelNEDTwist(rclcpp::PublisherBase::SharedPtr&, system_state_packet_t, velocity_standard_deviation_packet_t,rclcpp::Clock&);
void PublishVelENUTwist(rclcpp::PublisherBase::SharedPtr&, system_state_packet_t, velocity_standard_deviation_packet_t,rclcpp::Clock&);
void PublishVelBodyTwistFLU(rclcpp::PublisherBase::SharedPtr&, system_state_packet_t, body_velocity_packet_t, velocity_standard_deviation_packet_t,rclcpp::Clock&);
void PublishVelBodyTwistFRD(rclcpp::PublisherBase::SharedPtr&, system_state_packet_t, body_velocity_packet_t, velocity_standard_deviation_packet_t,rclcpp::Clock&);

