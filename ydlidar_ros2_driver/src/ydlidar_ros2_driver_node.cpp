/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node
 *
 *  Copyright 2017 - 2020 EAI TEAM
 *  http://www.eaibot.com
 *
 */

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include "src/CYdLidar.h"
#include <math.h>
#include <chrono>
#include <iostream>
#include <memory>
#include "sensor_msgs/msg/point_cloud.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>
#include <angles/angles.h>
#include <limits>

#define ROS2Verision "1.0.1"


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");

  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Current ROS Driver Version: %s\n", ((std::string)ROS2Verision).c_str());

  CYdLidar laser;
  std::string str_optvalue = "/dev/ydlidar";
  node->declare_parameter("port", str_optvalue);
  node->get_parameter("port", str_optvalue);
  ///lidar port
  laser.setlidaropt(LidarPropSerialPort, str_optvalue.c_str(), str_optvalue.size());

  ///ignore array
  str_optvalue = "";
  node->declare_parameter("ignore_array", str_optvalue);
  node->get_parameter("ignore_array", str_optvalue);
  laser.setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(), str_optvalue.size());

  std::string frame_id = "laser_frame";
  node->declare_parameter("frame_id", frame_id);
  node->get_parameter("frame_id", frame_id);

  //////////////////////int property/////////////////
  /// lidar baudrate
  int optval = 230400;
  node->declare_parameter("baudrate", optval);
  node->get_parameter("baudrate", optval);
  laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
  /// tof lidar
  optval = TYPE_TRIANGLE;
  node->declare_parameter("lidar_type", optval);
  node->get_parameter("lidar_type", optval);
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
  /// device type
  optval = YDLIDAR_TYPE_SERIAL;
  node->declare_parameter("device_type", optval);
  node->get_parameter("device_type", optval);
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
  /// sample rate
  optval = 9;
  node->declare_parameter("sample_rate", optval);
  node->get_parameter("sample_rate", optval);
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
  /// abnormal count
  optval = 4;
  node->declare_parameter("abnormal_check_count", optval);
  node->get_parameter("abnormal_check_count", optval);
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

  /// Intenstiy bit count
  optval = 8;
  node->declare_parameter("intensity_bit", optval);
  node->get_parameter("intensity_bit", optval);
  laser.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));
     
  //////////////////////bool property/////////////////
  /// fixed angle resolution
  bool b_optvalue = false;
  node->declare_parameter("fixed_resolution", b_optvalue);
  node->get_parameter("fixed_resolution", b_optvalue);
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
  /// rotate 180
  b_optvalue = true;
  node->declare_parameter("reversion", b_optvalue);
  node->get_parameter("reversion", b_optvalue);
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  /// Counterclockwise
  b_optvalue = true;
  node->declare_parameter("inverted", b_optvalue);
  node->get_parameter("inverted", b_optvalue);
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  b_optvalue = true;
  node->declare_parameter("auto_reconnect", b_optvalue);
  node->get_parameter("auto_reconnect", b_optvalue);
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
  /// one-way communication
  b_optvalue = false;
  node->declare_parameter("isSingleChannel", b_optvalue);
  node->get_parameter("isSingleChannel", b_optvalue);
  laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
  /// intensity
  b_optvalue = false;
  node->declare_parameter("intensity", b_optvalue);
  node->get_parameter("intensity", b_optvalue);
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  /// Motor DTR
  b_optvalue = false;
  node->declare_parameter("support_motor_dtr", b_optvalue);
  node->get_parameter("support_motor_dtr", b_optvalue);
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue = 180.0f;
  node->declare_parameter("angle_max", f_optvalue);
  node->get_parameter("angle_max", f_optvalue);
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  f_optvalue = -180.0f;
  node->declare_parameter("angle_min", f_optvalue);
  node->get_parameter("angle_min", f_optvalue);
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));

  int angle_min1 = 0;
  node->declare_parameter("angle_min1", angle_min1);
  node->get_parameter("angle_min1", angle_min1);
  float f_angle_min1 = angles::normalize_angle(angle_min1/180.0*M_PI);

  int angle_max1 = 90;
  node->declare_parameter("angle_max1", angle_max1);
  node->get_parameter("angle_max1", angle_max1);
  float f_angle_max1 = angles::normalize_angle(angle_max1/180.0*M_PI);

  int angle_min2 = 270;
  node->declare_parameter("angle_min2", angle_min2);
  node->get_parameter("angle_min2", angle_min2);
  float f_angle_min2 = angles::normalize_angle(angle_min2/180.0*M_PI);

  int angle_max2 = 360;
  node->declare_parameter("angle_max2", angle_max2);
  node->get_parameter("angle_max2", angle_max2);
  float f_angle_max2 = angles::normalize_angle(angle_max2/180.0*M_PI);

  /// unit: m
  f_optvalue = 64.f;
  node->declare_parameter("range_max", f_optvalue);
  node->get_parameter("range_max", f_optvalue);
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  f_optvalue = 0.1f;
  node->declare_parameter("range_min", f_optvalue);
  node->get_parameter("range_min", f_optvalue);
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  /// unit: Hz
  f_optvalue = 10.f;
  node->declare_parameter("frequency", f_optvalue);
  node->get_parameter("frequency", f_optvalue);
  laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));

  bool invalid_range_is_inf = false;
  node->declare_parameter("invalid_range_is_inf", invalid_range_is_inf);
  node->get_parameter("invalid_range_is_inf", invalid_range_is_inf);

  float point_dis_scale = 0.025;
  node->declare_parameter("point_dis_scale", point_dis_scale);
  node->get_parameter("point_dis_scale", point_dis_scale);

  int point_continues_num = 2;
  node->declare_parameter("point_continues_num", point_continues_num);
  node->get_parameter("point_continues_num", point_continues_num);
  point_continues_num = point_continues_num < 2 ? 2 : point_continues_num;

  bool ret = laser.initialize();
  if (ret) {
    ret = laser.turnOn();
  } else {
    RCLCPP_ERROR(node->get_logger(), "%s\n", laser.DescribeError());
  }
  
  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
  auto pc_pub = node->create_publisher<sensor_msgs::msg::PointCloud>("point_cloud", rclcpp::SensorDataQoS());
  
  auto stop_scan_service =
    [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
  {
    return laser.turnOff();
  };

  auto stop_service = node->create_service<std_srvs::srv::Empty>("stop_scan",stop_scan_service);

  auto start_scan_service =
    [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
  {
    return laser.turnOn();
  };

  auto start_service = node->create_service<std_srvs::srv::Empty>("start_scan",start_scan_service);

  rclcpp::WallRate loop_rate(20);

  while (ret && rclcpp::ok()) {

    LaserScan scan;//

    if (laser.doProcessSimple(scan)) {

      auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
      auto pc_msg = std::make_shared<sensor_msgs::msg::PointCloud>();

      scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
      scan_msg->header.stamp.nanosec =  scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
      scan_msg->header.frame_id = frame_id;
      pc_msg->header = scan_msg->header;
      scan_msg->angle_min = scan.config.min_angle;
      scan_msg->angle_max = scan.config.max_angle;
      scan_msg->angle_increment = scan.config.angle_increment;
      scan_msg->scan_time = scan.config.scan_time;
      scan_msg->time_increment = scan.config.time_increment;
      scan_msg->range_min = scan.config.min_range;
      scan_msg->range_max = scan.config.max_range;
      
      int size = (scan.config.max_angle - scan.config.min_angle)/ scan.config.angle_increment + 1;
      scan_msg->ranges.resize(size, std::numeric_limits<float>::infinity());
      scan_msg->intensities.resize(size, std::numeric_limits<float>::infinity());

      pc_msg->channels.resize(2);
      int idx_intensity = 0;
      pc_msg->channels[idx_intensity].name = "intensities";
      int idx_timestamp = 1;
      pc_msg->channels[idx_timestamp].name = "stamps";

      int index_range_min1 = std::ceil((f_angle_min1 - scan.config.min_angle)/scan.config.angle_increment);
      int index_range_max1 = std::ceil((f_angle_max1 - scan.config.min_angle)/scan.config.angle_increment);
      int index_range_min2 = std::ceil((f_angle_min2 - scan.config.min_angle)/scan.config.angle_increment);
      int index_range_max2 = std::ceil((f_angle_max2 - scan.config.min_angle)/scan.config.angle_increment);

      RCLCPP_INFO_ONCE(node->get_logger(), "min1: %f, max1: %f", f_angle_min1, f_angle_max1);
      RCLCPP_INFO_ONCE(node->get_logger(), "min2: %f, max2: %f", f_angle_min2, f_angle_max2);
      RCLCPP_INFO_ONCE(node->get_logger(), "index_min1: %d, index_max1: %d", index_range_min1, index_range_max1);
      RCLCPP_INFO_ONCE(node->get_logger(), "index_min2: %d, index_max2: %d", index_range_min2, index_range_max2);

      for(size_t i=0; i < scan.points.size(); i++) {
        int index = std::ceil((scan.points[i].angle - scan.config.min_angle)/scan.config.angle_increment);
        RCLCPP_DEBUG(node->get_logger(), "i: %zd, index: %d, angle: %f, angle_min: %f, angle_increament: %f", i, index, scan.points[i].angle, scan.config.min_angle, scan.config.angle_increment);
        if(index >=0 && index < size) {
          if ((index >= index_range_min1 && index <= index_range_max1) || (index >= index_range_min2  && index <= index_range_max2))
          {
            RCLCPP_DEBUG(node->get_logger(), "ignore index: %d", index);
            continue;
          }
          else
          {
            if (scan.points[i].range >= scan.config.min_range) 
            {
                  scan_msg->ranges[index] = scan.points[i].range;
                  scan_msg->intensities[index] = scan.points[i].intensity;
            }
          }
        }

        if (scan.points[i].range >= scan.config.min_range &&
            scan.points[i].range <= scan.config.max_range) {
          geometry_msgs::msg::Point32 point;
          point.x = scan.points[i].range * cos(scan.points[i].angle);
          point.y = scan.points[i].range * sin(scan.points[i].angle);
          point.z = 0.0;
          pc_msg->points.push_back(point);
          pc_msg->channels[idx_intensity].values.push_back(scan.points[i].intensity);
          pc_msg->channels[idx_timestamp].values.push_back(i * scan.config.time_increment);
        }

      }

      // 相临range数据值大小差值要相近
      // static int frame_id = 0;
      // RCLCPP_DEBUG(node->get_logger(), "--------------- frame %d ---------------", frame_id++);
      // int num_delete = 0;
      // for (size_t i = 1; i < scan_msg->ranges.size() - 2; i++)
      // {
      //   RCLCPP_DEBUG(node->get_logger(), "front: %f, current: %f, back: %f ", scan_msg->ranges[i-1], scan_msg->ranges[i], scan_msg->ranges[i+1]);
      //   float delta_l, delta_r;
      //   float point_dis_max;
      //   if (std::isinf(scan_msg->ranges[i]))
      //   {
      //     continue;
      //   }
      //   if (std::isinf(scan_msg->ranges[i-1]) && std::isinf(scan_msg->ranges[i+1]))
      //   {
      //     RCLCPP_DEBUG(node->get_logger(), "l_r inf, delete index: %zd", i);
      //     scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
      //     num_delete++;
      //     continue;
      //   }
      //   if (std::isinf(scan_msg->ranges[i-1]))
      //   {
      //     delta_r = std::fabs(scan_msg->ranges[i+1] - scan_msg->ranges[i]);
      //     point_dis_max = scan_msg->ranges[i] * point_dis_scale;
      //     if (delta_r > point_dis_max)
      //     {
      //       RCLCPP_DEBUG(node->get_logger(), "l inf, r: %f > %f, delete index: %zd",delta_r, point_dis_max, i);
      //       scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
      //       num_delete++;
      //       continue;
      //     } 
      //   }
      //   if (std::isinf(scan_msg->ranges[i+1]))
      //   {
      //     delta_l = std::fabs(scan_msg->ranges[i-1] - scan_msg->ranges[i]);
      //     point_dis_max = scan_msg->ranges[i] * point_dis_scale;
      //     if (delta_l > point_dis_max)
      //     {
      //       RCLCPP_DEBUG(node->get_logger(), "r inf, l: %f > %f, delete index: %zd",delta_l, point_dis_max, i);
      //       scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
      //       num_delete++;
      //       continue;
      //     } 
      //   }
      //   delta_l = std::fabs(scan_msg->ranges[i-1] - scan_msg->ranges[i]);
      //   delta_r = std::fabs(scan_msg->ranges[i+1] - scan_msg->ranges[i]);
      //   if (delta_l <= point_dis_max || delta_r <= point_dis_max)
      //   {
      //     continue;
      //   }
      //   else
      //   {
      //     scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
      //     RCLCPP_DEBUG(node->get_logger(), "l_r > %f, delta_l: %f, delta_r: %f delete index: %zd", point_dis_max, delta_l, delta_r, i);
      //     num_delete++;
      //   }
      // }

      // keep n continues ranges data
      int ranges_size = (int)(scan_msg->ranges.size());
      float point_dis_max;
      for (int i = 0; i < ranges_size;)
      {
        RCLCPP_DEBUG(node->get_logger(), "i: %d", i);
        if (std::isinf(scan_msg->ranges[i]))
        {
         RCLCPP_DEBUG(node->get_logger(), "inf, i++");
          i++;
          continue;
        }
        else
        {
          int index_min, index_max;
          int offset = point_continues_num - 1;
          int continues_l = 0, continues_r = 0;
          index_min = std::max(i - offset, 0);
          index_max = std::min(i + offset, ranges_size - 1);
          int index_l, index_r;

          point_dis_max = scan_msg->ranges[i] * point_dis_scale;

          // print value
          for (int ii = index_min; ii <= index_max; ii++)
          {
            RCLCPP_DEBUG(node->get_logger(), "index: %d, value: %s", ii, std::isinf(scan_msg->ranges[ii])?"inf": (std::ostringstream()<<scan_msg->ranges[ii]).str().c_str());
          }

          if (index_min < i)
          {
            for (index_l = i - 1; index_l >= index_min; index_l--)
            {
              if (std::isinf(scan_msg->ranges[index_l]))
              {
                break;
              }
              else
              {
                float delta = std::abs(scan_msg->ranges[i] - scan_msg->ranges[index_l]);
                RCLCPP_DEBUG(node->get_logger(), "point_dis_max: %f, delta: %f", point_dis_max, delta);
                if (delta < point_dis_max)
                {
                  continues_l++;
                }
                else
                {
                  break;
                }                
              }
            }
          }
          if (continues_l == offset)
          { 
            RCLCPP_DEBUG(node->get_logger(), "continuse_l satisfied, i++", i);
            i++;
            continue;
          }
          else
          {
            if (index_max > i)
            {
              for (index_r = i + 1; index_r <= index_max; index_r++)
              {
                if (std::isinf(scan_msg->ranges[index_r]))
                {
                  break;
                }
                else
                {
                  float delta = std::abs(scan_msg->ranges[i] - scan_msg->ranges[index_r]);
                  RCLCPP_DEBUG(node->get_logger(), "point_dis_max: %f, delta: %f", point_dis_max, delta);
                  if (delta < point_dis_max)
                  {
                    continues_l++;
                  }
                  else
                  {
                    break;
                  }    
                }
              }
              // fix bug for index_r
              index_r = index_r == index_max + 1 ? index_max : index_r;

              if ((continues_l + continues_r) >= offset)
              {
                RCLCPP_DEBUG(node->get_logger(), "continuse_l+r satisfied, i = index_r + 1, index_r: %d", index_r);
                i = index_r + 1;
                continue;
              }
              else
              {
                for (int j = i; j <= index_r; j++)
                {
                  RCLCPP_DEBUG(node->get_logger(), "deleta index %d for continues.", j);
                  scan_msg->ranges[j] = std::numeric_limits<float>::infinity();
                }
                RCLCPP_DEBUG(node->get_logger(), "continuse_l+r not satisfied, i = index_r + 1, index_r: %d", index_r);
                i = index_r + 1;
              }
            } // end of index_max > i
            else
            {
              RCLCPP_DEBUG(node->get_logger(), "index_max not > i, index_max:%d", index_max);
              i++;
              continue;
            }            
          }

        } // end of else, std::isinf(scan_msg->ranges[i])        

      } // end of for
      // RCLCPP_DEBUG(node->get_logger(), "delete %d points.", num_delete);

      laser_pub->publish(*scan_msg);
      pc_pub->publish(*pc_msg);

    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to get scan");
    }
    if(!rclcpp::ok()) {
      break;
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }


  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Now YDLIDAR is stopping .......");
  laser.turnOff();
  laser.disconnecting();
  rclcpp::shutdown();

  return 0;
}
