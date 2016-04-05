/******************************************************************************
   STDR Simulator - Simple Two DImensional Robot Simulator
   Copyright (C) 2013 STDR Simulator
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
   
   Authors : 
   * Manos Tsardoulias, etsardou@gmail.com
   * Aris Thallas, aris.thallas@gmail.com
   * Chris Zalidis, zalidis@gmail.com 
******************************************************************************/

#include <stdr_robot/sensors/battery.h>
#define BATTERY_CONSUMPTION 1

namespace stdr_robot {
  
  /**
  @brief Default constructor
  @param map [const nav_msgs::OccupancyGrid&] An occupancy grid map
  @param msg [const stdr_msgs::BatterySensorMsg&] The sensor description message
  @param name [const std::string&] The sensor frame id without the base
  @param n [ros::NodeHandle&] The ROS node handle
  @return void
  **/ 
  Battery::Battery(
    const nav_msgs::OccupancyGrid& map,
    const stdr_msgs::BatterySensorMsg& msg, 
    const std::string& name,
    ros::NodeHandle& n)
    : Sensor(map, name, n, msg.pose, msg.frame_id, msg.frequency)
  {
    _description = msg;

    _publisher = n.advertise<stdr_msgs::BatterySensorMeasurement>
      ( "battery", 1 );
  }

  /**
  @brief Default destructor
  @return void
  **/ 
  Battery::~Battery()
  {

  }

  /**
  @brief Updates the sensor measurements
  @return void
  **/ 
  void Battery::updateSensorCallback() 
  {
	stdr_msgs::BatterySensorMeasurement measuredSourcesMsg;
    measuredSourcesMsg.header.stamp = ros::Time::now();
    measuredSourcesMsg.header.frame_id = _namespace + "_" + _description.frame_id;
	measuredSourcesMsg.consumption = BATTERY_CONSUMPTION;
	_publisher.publish( measuredSourcesMsg );
  }
}  // namespace stdr_robot
