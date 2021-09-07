/*
 * Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <iostream>  
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <ament_index_cpp/get_resource.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.h"
#include "nav_msgs/msg/path.hpp"
//#include "nav2_costmap_2d/costmap_2d_ros.hpp"
      
int image_num = 0;
using namespace std;
class MyNode : public rclcpp::Node{
public:
    MyNode() : Node("plan_receive")
    {
    }
    //void callback(const nav_msgs::msg::OccupancyGrid::SharedPtr occupancygrid)
    void callback(const nav_msgs::msg::Path::SharedPtr path_receive)
    {
        ;
        std::cout<<"收到了："<<path_receive->poses.size()<<std::endl;
        int num = path_receive->poses.size();
        geometry_msgs::msg::PoseStamped * value = NULL; //= new geometry_msgs::msg::Pose[num]; // 为变量请求内存
        value = path_receive->poses.data();
        //value[0].pose.position;
        //memcpy(value,path_receive->poses.data,num);
        
        for(int i=0;i<num;i++)
        {
            std::cout<<value[i].pose.position.x<<" "<<value[i].pose.position.y<<" " << value[i].pose.position.z<<std::endl;
        }
        
        /*x`
        height = occupancygrid->info.height;
        width = occupancygrid->info.width;
        std::vector<int8_t> data = occupancygrid->data;
        int8_t *int8_data = new int8_t[data.size()];
        unsigned char* color_date = new unsigned char[data.size()];

        memcpy(int8_data, &data[0], data.size()*sizeof(int8_t));
        for(int i=0;i<data.size();i++)
        {
            if(int8_data[i]==-1)
            {
                color_date[i]=255;
            }else{
                color_date[i]=255*(100-int8_data[i])/100;
            }
        }
        //memcpy(color_date, &data[0], data.size()*sizeof(unsigned char));
        auto max = std::max_element(data.begin(),data.end());
        auto min = std::min_element(data.begin(),data.end());
        std::cout<<width<<" "<<height<<" "<<int(*min)<<" "<<int(*max)<<" "<< occupancygrid->info.origin.position.x<< "," \
        << occupancygrid->info.origin.position.y<< "," << occupancygrid->info.origin.position.z<< "\n";

        cv::Mat Img(height, width, CV_8UC1,color_date);//
        cv::imshow("test",Img);
        cv::waitKey(1);
        */
        //std::cout<<"receive ok ???"<<std::endl;
    }
    void callback_pointcloud2(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud2)
    {
        std::cout<<"success"<<std::endl;
    }
    void callback_odom(const nav_msgs::msg::Odometry::SharedPtr pose)
    {
        std::cout<<"pose success"<<std::endl;
    }
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscriber;
    //rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_subscriber;
    //rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
private:
    int height;
    int width;
    //rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscriber;
};
int main (int argc, char** argv)
{ 
  
  rclcpp::init(argc, argv);
    auto node_plan_receive = std::make_shared<MyNode>();
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    node_plan_receive->subscriber = node_plan_receive->create_subscription<nav_msgs::msg::Path>("/plan", qos, std::bind(&MyNode::callback, node_plan_receive, std::placeholders::_1));
    //node_plan_receive->pointcloud2_subscriber = node_plan_receive->create_subscription<sensor_msgs::msg::PointCloud2>("/scan_matched_points2", qos, std::bind(&MyNode::callback_pointcloud2, node_occupancy_grid, std::placeholders::_1));
    //node_plan_receive->odom_subscriber = node_plan_receive->create_subscription<nav_msgs::msg::Odometry>("/odom", qos, std::bind(&MyNode::callback_odom, node_occupancy_grid, std::placeholders::_1));
    //auto node = std::make_shared<nav2_costmap_2d::Costmap2DROS>("costmap");
    //rclcpp::spin(node->get_node_base_interface());
    rclcpp::spin(node_plan_receive);
    rclcpp::shutdown();
    /*
  auto node = rclcpp::Node::make_shared("realsene_image_raw_sub");
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_im = node->create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw", qos, imCallBack);
  rclcpp::WallRate loop_rate(1);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  while(rclcpp::ok())
  {
    loop_rate.sleep();
    executor.spin_once(std::chrono::seconds(0));
  }
    */
  return 0;
}  

