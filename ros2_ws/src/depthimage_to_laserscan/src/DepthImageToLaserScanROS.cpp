/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Chad Rockey
 */

#include <functional>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <depthimage_to_laserscan/DepthImageToLaserScanROS.hpp>

namespace depthimage_to_laserscan{

DepthImageToLaserScanROS::DepthImageToLaserScanROS(const rclcpp::NodeOptions & options): rclcpp::Node("depthimage_to_laserscan", options){
  auto qos = rclcpp:: SystemDefaultsQoS();
  //std::cout<<qos<<std::endl;
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("depth_camera_info", qos,
      std::bind(
        &DepthImageToLaserScanROS::infoCb, this,
        std::placeholders::_1));

  depth_image_sub_ =
    this->create_subscription<sensor_msgs::msg::Image>("depth", qos,
      std::bind(&DepthImageToLaserScanROS::depthCb, this, std::placeholders::_1));

  scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", qos);

  //float scan_time = this->declare_parameter("scan_time", 0.033);
    float scan_time = this->declare_parameter("scan_time", 0.033);
  dtl_.set_scan_time(scan_time);

  float range_min = this->declare_parameter("range_min", 0.45);
  float range_max  = this->declare_parameter("range_max", 10.0);
  dtl_.set_range_limits(range_min, range_max);

  int scan_height = this->declare_parameter("scan_height", 1);
  dtl_.set_scan_height(scan_height);

  std::string output_frame = this->declare_parameter("output_frame", "camera_depth_frame");
  dtl_.set_output_frame(output_frame);
  std::cout<<"init success"<<std::endl;
}

DepthImageToLaserScanROS::~DepthImageToLaserScanROS(){
}

void DepthImageToLaserScanROS::infoCb(sensor_msgs::msg::CameraInfo::SharedPtr info){
  cam_info_ = info;
  /*
    cam_info_->height=480,cam_info_->width=640;
    cam_info_->binning_x = 0, cam_info_->binning_y=0;

    //memset(cam_info_->k, 0.0, 9 * sizeof(double));
    cam_info_->k[0]=382.659,cam_info_->k[1]=0,cam_info_->k[2]=322.194,cam_info_->k[3]=0;
    cam_info_->k[4]=382.659,cam_info_->k[5]=236.368,cam_info_->k[6]=0,cam_info_->k[7]=0,cam_info_->k[8]=1;

    //memset(cam_info_->r, 0.0, 9 * sizeof(double));
    cam_info_->r[0]=1,cam_info_->r[1]=0,cam_info_->r[2]=0,cam_info_->r[3]=0,cam_info_->r[4]=1;
    cam_info_->r[5]=0,cam_info_->r[6]=0,cam_info_->r[7]=0,cam_info_->r[8]=1;

    //memset(cam_info_->p, 0.0, 12 * sizeof(double));
    cam_info_->p[0]=382.659,cam_info_->p[1]=0,cam_info_->p[2]=322.194,cam_info_->p[3]=0,cam_info_->p[4]=0,cam_info_->p[5]=382.659;
    cam_info_->p[6]=236.368;cam_info_->p[7]=0,cam_info_->p[8]=0,cam_info_->p[9]=0,cam_info_->p[10]=1,cam_info_->p[11]=0;


    struct timespec tn;
    clock_gettime(CLOCK_REALTIME, &tn);
    cam_info_->header.stamp.sec = tn.tv_sec,cam_info_->header.stamp.nanosec = tn.tv_nsec;
    cam_info_->header.frame_id = "camera_depth_optical_frame";
    cam_info_->distortion_model = "plumb_bob";
    */
    /*
    std::cout<<cam_info_->header.stamp.sec<<" "<<cam_info_->header.stamp.nanosec<<" "<<cam_info_->header.frame_id<<std::endl;
    std::cout<<cam_info_->distortion_model<<std::endl;
    std::cout<<cam_info_->d<<std::endl;
  std::cout<<cam_info_->height<<" "<<cam_info_->width<<std::endl;
  std::cout<<cam_info_->binning_x<<std::endl;
  std::cout<<cam_info_->binning_y<<std::endl;
  for(int i=0;i<9;i++)
  {
      std::cout<<cam_info_->k[i]<<" ";
  }
  std::cout<<std::endl;
  for(int i=0;i<9;i++)
  {
      std::cout<<cam_info_->r[i]<<" ";
  }
  std::cout<<std::endl;
  for(int i=0;i<12;i++)
  {
      std::cout<<cam_info_->p[i]<<" ";
  }
  std::cout<<std::endl;
   */
}
/*
void DepthImageToLaserScanROS::depthCb2(const sensor_msgs::msg::Image::SharedPtr image) {
    if (nullptr == cam_info_) {
        RCLCPP_INFO(get_logger(), "No camera info, skipping point cloud squash");
        return;
    } else {
        std::cout << "yes" << std::endl;
    }
}
 */
void DepthImageToLaserScanROS::depthCb(const sensor_msgs::msg::Image::SharedPtr image){
  if (nullptr == cam_info_){
    RCLCPP_INFO(get_logger(), "No camera info, skipping point cloud squash");
    return;
  }else{
      //std::cout<<"recive depth image"<<std::endl;
  }

  try{
    sensor_msgs::msg::LaserScan::UniquePtr scan_msg = dtl_.convert_msg(image, cam_info_);
    //std::cout<<"转换成功并发布"<<std::endl;
    //scan_msg->header.stamp =rclcpp::Clock().now();
    scan_pub_->publish(std::move(scan_msg));
  }
  catch (std::runtime_error& e){
    RCLCPP_ERROR(get_logger(), "Could not convert depth image to laserscan: %s", e.what());
  }
}
}  // namespace depthimage_to_laserscan

RCLCPP_COMPONENTS_REGISTER_NODE(depthimage_to_laserscan::DepthImageToLaserScanROS)
