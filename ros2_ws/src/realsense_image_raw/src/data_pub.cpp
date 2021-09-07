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
#include <cstdlib>
#include <string>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>


#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <sensor_msgs/msg/image.hpp>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <rcutils/logging_macros.h>
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/buffer_core.h"
#include "tf2/convert.h"

//#include <geometry_msgs/TransformStamped.h>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "eigen3/Eigen/Geometry"
#include "librealsense2/rs.hpp"
#include "realsense_msgs/msg/imu_info.hpp"
//#include "realsense/rs_constants.hpp"

#include <ament_index_cpp/get_resource.hpp>
#include <unistd.h>
#include <limits.h>
#include <vector>
#include <dirent.h>
#include <iostream>
//#include <Python.h>
//#include <C_Detect.h>
//#include <numpy/numpyconfig.h>
//#include <numpy/arrayobject.h>
//#include <numpy/ndarraytypes.h>
#define PLAYRATE 1
#define IMAGENUM 2000
#define MAXSIZE 20
#define MAX_NUM 6000
#define SERVERPORT 10091
#define ADDR "127.0.0.1" //在本机测试用这个地址，如果连接其他电脑需要更换IP
using namespace std;
std::string getConfigPath()
{
  std::string content;
  std::string prefix_path;
  ament_index_cpp::get_resource("packages", "realsense_image_raw", content, &prefix_path);
  return prefix_path;
}

typedef struct INFOMATION
{
    double x;
    double y;
    double z;
    double r_x;
    double r_y;
    double r_z;
    double r_w;
};

vector<string> split(const string& str, const string delim) {
    vector<string> res;
    if("" == str) return res;
    //先将要切割的字符串从string类型转换为char*类型
    char * strs = new char[str.length() + 1] ; //不要忘了
    strcpy(strs, str.c_str());

    char * d = new char[delim.length() + 1];
    strcpy(d, delim.c_str());

    char *p = strtok(strs, d);
    while(p) {
        string s = p; //分割得到的字符串转换为string类型
        res.push_back(s); //存入结果数组
        p = strtok(NULL, d);
    }
    return res;
}
std::vector<nav_msgs::msg::Odometry> vec_msg_odom;
std::vector<geometry_msgs::msg::TransformStamped> TransformStamped_msg;

void getPose(auto node_pub)
{

    //rclcpp::Node & node_;
    //std::ifstream in("/media/gewenhao/disk3/realsense/2019_07_10/1594435370/T265output.txt");
    //std::ifstream in("/media/gewenhao/disk3/realsense/2019_07_10/1594436543/T265output.txt");
    std::ifstream in("/media/gwh/新加卷/realsense/fit一层/T265output.txt");

    std::string filename;
    std::string line;
    if(in) // 有该文件
    {
        int i=0;

        while (getline (in, line)) // line中不包括每行的换行符
        {
            i++;
            //std::cout<<"??"<<std::endl;
            if(i>MAX_NUM)
                break;
            std::vector<std::string> sv;
            sv = split(line, ", ");
            string new_1,new_2;

            /*
            int pos = sv[8].rfind('.');
            int pos_e = sv[8].find('e',pos);
            //if(pos_e==-1)
            //{
                //std::cout<<"no_e ";
            
            int pos_1 = pos-1;
            if(sv[8][pos-2]=='-')
            {
                new_2.push_back(sv[8][pos-2]);
                new_2.push_back(sv[8][pos-1]);
                pos_1 = pos-2;
            }else{
                new_2.push_back(sv[8][pos-1]);
            }
            for(int i=pos;i<sv[8].length();i++)
            {
                new_2.push_back(sv[8][i]);
            }
            for(int i=0;i<pos_1;i++)
            {
                new_1.push_back(sv[8][i]);
            }
            */
            //double n1 = stod(new_1), n2 = stod(new_2);
            //std::cout<<n1<<" "<<n2<<std::endl;

            //std::cout<<sv[18]<<std::endl;
            //int tracker_confidence = std::stoi(sv[18]);
            int tracker_confidence = std::stoi(sv[19]);
            //std::cout<<tracker_confidence<<std::endl;
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.pose.position.x = -stod(sv[2]);//-z
            pose_msg.pose.position.y = -stod(sv[0]);//-x
            pose_msg.pose.position.z = stod(sv[1]); //y

            pose_msg.pose.orientation.x = -stod(sv[11]); //-z
            pose_msg.pose.orientation.y = -stod(sv[9]); //-x
            pose_msg.pose.orientation.z = stod(sv[10]);  //y
            pose_msg.pose.orientation.w = stod(sv[12]); //w
            /*
            pose_msg.pose.orientation.y = -stod(new_2); //-x
            pose_msg.pose.orientation.z = stod(sv[9]);  //y
            pose_msg.pose.orientation.w = stod(sv[11]); //w
            */

            static tf2_ros::TransformBroadcaster br(node_pub);
            geometry_msgs::msg::TransformStamped msg;
            //msg.header.stamp = time;
            //header.frame_id = DEFAULT_ODOM_FRAME_ID;
            msg.header.frame_id = "odom_frame";
            //msg.child_frame_id = OPTICAL_FRAME_ID.at(type_index);
            msg.child_frame_id = "camera_pose_optical_frame";
            //std::cout<<msg.header.frame_id<<" "<<msg.child_frame_id<<std::endl;
            msg.transform.translation.x = pose_msg.pose.position.x;
            msg.transform.translation.y = pose_msg.pose.position.y;
            msg.transform.translation.z = pose_msg.pose.position.z;
            msg.transform.rotation.x = pose_msg.pose.orientation.x;
            msg.transform.rotation.y = pose_msg.pose.orientation.y;
            msg.transform.rotation.z = pose_msg.pose.orientation.z;
            msg.transform.rotation.w = pose_msg.pose.orientation.w;
            


            TransformStamped_msg.push_back(msg);
            //br.sendTransform(msg);

            //double linear_accel_cov_ = DEFAULT_LINEAR_ACCEL_COV;
            double linear_accel_cov_ = 0.01;
            //double angular_velocity_cov_ = DEFAULT_ANGULAR_VELOCITY_COV;
            double angular_velocity_cov_ = 0.01;

            //double cov_pose(linear_accel_cov_ * pow(10, 3-(int)pose.tracker_confidence));
            double cov_pose(linear_accel_cov_ * pow(10, 3-(int)tracker_confidence));
            //double cov_twist(angular_velocity_cov_ * pow(10, 1-(int)pose.tracker_confidence));
            double cov_twist(angular_velocity_cov_ * pow(10, 1-(int)tracker_confidence));

            geometry_msgs::msg::Vector3Stamped v_msg;
            //tf2::Vector3 tfv(-pose.velocity.z,-pose.velocity.x, pose.velocity.y);
            tf2::Vector3 tfv(-stod(sv[5]),-stod(sv[3]), stod(sv[4]));
            tf2::Quaternion q(-msg.transform.rotation.x,-msg.transform.rotation.y,-msg.transform.rotation.z,msg.transform.rotation.w);
            tfv = tf2::quatRotate(q,tfv);
            v_msg.vector.x = tfv.x();
            v_msg.vector.y = tfv.y();
            v_msg.vector.z = tfv.z();

            geometry_msgs::msg::Vector3Stamped om_msg;
            //tf2::Vector3 tfo(-pose.angular_velocity.z,-pose.angular_velocity.x, pose.angular_velocity.y);
            //tf2::Vector3 tfo(-stod(sv[14]),-stod(sv[12]), stod(sv[13]) );
            tf2::Vector3 tfo(-stod(sv[15]),-stod(sv[13]), stod(sv[14]) );
            tfo = tf2::quatRotate(q,tfo);
            om_msg.vector.x = tfo.x();
            om_msg.vector.y = tfo.y();
            om_msg.vector.z = tfo.z();

            nav_msgs::msg::Odometry odom_msg;

            //odom_msg.header.frame_id = DEFAULT_ODOM_FRAME_ID;
            odom_msg.header.frame_id = "odom_frame";
            //odom_msg.child_frame_id = OPTICAL_FRAME_ID.at(type_index);
            odom_msg.child_frame_id = "camera_pose_optical_frame";
            //std::cout<<odom_msg.child_frame_id<<std::endl;

            //odom_msg.header.stamp = time;

            odom_msg.pose.pose = pose_msg.pose;
            odom_msg.pose.covariance = {cov_pose, 0, 0, 0, 0, 0,
                                        0, cov_pose, 0, 0, 0, 0,
                                        0, 0, cov_pose, 0, 0, 0,
                                        0, 0, 0, cov_twist, 0, 0,
                                        0, 0, 0, 0, cov_twist, 0,
                                        0, 0, 0, 0, 0, cov_twist};
            odom_msg.twist.twist.linear = v_msg.vector;
            odom_msg.twist.twist.angular = om_msg.vector;
            odom_msg.twist.covariance ={cov_pose, 0, 0, 0, 0, 0,
                                        0, cov_pose, 0, 0, 0, 0,
                                        0, 0, cov_pose, 0, 0, 0,
                                        0, 0, 0, cov_twist, 0, 0,
                                        0, 0, 0, 0, cov_twist, 0,
                                        0, 0, 0, 0, 0, cov_twist};

            vec_msg_odom.push_back(odom_msg);
            /*
            auto p_frame = frame.as<rs2::pose_frame>();
            rs2_pose pose = p_frame.get_pose_data();
                                        */
            //}else{
            //    std::cout<<"yes_e ";
            //    std::cout<<pos<<" "<<sv[8]<<std::endl;
            //}

            //std::cout << line << std::endl;
        }
        std::cout<<"while完就走了？"<<std::endl;
    }
    else // 没有该文件
    {
        std::cout <<"no such file" << std::endl;
    }
}

/*
void occupancy_grid_deal(const nav_msgs::msg::OccupancyGrid::SharedPtr occupancygrid)
{
    std::cout<<"receive ok ???"<<std::endl;
}
*/
int main(int argc, char** argv)
{
    //auto node_dp = rclcpp::Node::make_shared("realsense_depth_raw_pub");
    //auto node_info = rclcpp::Node::make_shared("realsense_depth_info");
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("realsense_d435");
  //auto node_occupancy_grid = rclcpp::Node::make_shared("occupancy_grid");
  auto node_odom = rclcpp::Node::make_shared("odom_pub");
    //auto node_occupancy_grid = std::make_shared<MyNode>();
  //std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  auto pub_im = node->create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw", 10);
  //auto pub_dp = node_dp->create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_raw", 10);
  //auto pub_info = node_info->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/depth/info",10);
  auto pub_dp = node->create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_raw", 10);
  auto pub_info = node->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/depth/info",10);
  auto pub_odom = node_odom->create_publisher<nav_msgs::msg::Odometry>("/odom",10);
  //static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster;
    static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
    static tf2_ros::TransformBroadcaster br(node_odom);


    getPose(node_odom);

    std::cout<<vec_msg_odom.size()<<std::endl;
    //return 0;
  std::string strFilePath;
  std::string strDepthFilePath;
  char imageFile[MAXSIZE];
  char depthFile[MAXSIZE];
  std::vector<sensor_msgs::msg::Image::SharedPtr> vec_msg_im;
  std::vector<sensor_msgs::msg::Image::SharedPtr> vec_msg_dp;

  rclcpp::WallRate loop_rate(PLAYRATE);
  std::cout<<"[INFO] []: Start to punlish Rgb! -- "<<"PLAY_RATE: "<<PLAYRATE<<", IMAGE_NUM: "<<IMAGENUM<<std::endl;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

    executor.add_node(node_odom);



  sensor_msgs::msg::CameraInfo cam_info_;
    std::cout<<"3"<<std::endl;
    cam_info_.header.frame_id = std::string("d435_link");
    std::cout<<"4"<<std::endl;
  cam_info_.height=480,cam_info_.width=640;
    std::cout<<"9"<<std::endl;
  cam_info_.binning_x = 0, cam_info_.binning_y=0;
    std::cout<<"5"<<std::endl;
  cam_info_.k[0]=382.659,cam_info_.k[1]=0,cam_info_.k[2]=322.194,cam_info_.k[3]=0;
  cam_info_.k[4]=382.659,cam_info_.k[5]=236.368,cam_info_.k[6]=0,cam_info_.k[7]=0,cam_info_.k[8]=1;
    std::cout<<"6"<<std::endl;
  cam_info_.r[0]=1,cam_info_.r[1]=0,cam_info_.r[2]=0,cam_info_.r[3]=0,cam_info_.r[4]=1;
  cam_info_.r[5]=0,cam_info_.r[6]=0,cam_info_.r[7]=0,cam_info_.r[8]=1;

  cam_info_.p[0]=382.659,cam_info_.p[1]=0,cam_info_.p[2]=322.194,cam_info_.p[3]=0,cam_info_.p[4]=0,cam_info_.p[5]=382.659;
  cam_info_.p[6]=236.368;cam_info_.p[7]=0,cam_info_.p[8]=0,cam_info_.p[9]=0,cam_info_.p[10]=1,cam_info_.p[11]=0;

  //cam_info_.header.frame_id = "camera_depth_optical_frame";
  cam_info_.distortion_model = "plumb_bob";
    std::cout<<"7"<<std::endl;

    cam_info_.header.stamp = rclcpp::Clock().now();
    pub_info->publish(cam_info_);
  //sensor_msgs::msg::CameraInfo::SharedPtr cam_info_p = cam_info_.;

    //strFilePath = "/media/gewenhao/disk3/realsense/2019_07_10/1594435370/up_color_transfrom";
    //strDepthFilePath= "/media/gewenhao/disk3/realsense/2019_07_10/1594435370/up_depth";


    //strFilePath = "/media/gewenhao/disk3/realsense/2019_07_10/1594436543/up_color";
    strFilePath = "/media/gwh/新加卷/realsense/fit一层/up_color";
    //strFilePath = "/media/gewenhao/disk3/realsense/2019_07_10/1594436543/up_color_transfrom";
    //strDepthFilePath= "/media/gewenhao/disk3/realsense/2019_07_10/1594436543/up_depth";
    strDepthFilePath= "/media/gwh/新加卷/realsense/fit一层/up_depth";



    /* socket 注释
  int sock;
  struct sockaddr_in serv_addr;
  
  sock = socket(PF_INET, SOCK_STREAM, 0);
  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = inet_addr(ADDR);  // 注释1
  serv_addr.sin_port = htons(SERVERPORT);
  if(connect(sock, (struct sockaddr*) &serv_addr, sizeof(serv_addr))==-1){ // 注释2
        cout << "connect error\n";
        return -1;
    }
    else{
        cout << "connected ...\n" << endl;  //注释3
    }

  INFOMATION infomation;
  char temp[100];    //传送的字符串
*/
  while(rclcpp::ok()) // loop play
  {

      std::cout<<"success"<<std::endl;
      for(int i = 0;i < IMAGENUM;i++)
      {
          std::sprintf(imageFile, "/%05d.png", i);
          std::string strFilePath_tmp = strFilePath + imageFile;
          std::string strDepthFilePath_tmp = strDepthFilePath + imageFile;
          cv::Mat image = cv::imread(strFilePath_tmp, 1);
          cv::Mat image_depth = cv::imread(strDepthFilePath_tmp, CV_LOAD_IMAGE_UNCHANGED);
          sensor_msgs::msg::Image::SharedPtr msg_im = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
          sensor_msgs::msg::Image::SharedPtr msg_dp = cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, image_depth).toImageMsg();
          msg_im->header.frame_id = std::string("d435_link");
          msg_dp->header.frame_id = std::string("d435_link");
          if(image.empty()) {
              std::cout << "Empty Image!" << std::endl;
              return 2;
          }
          if(image_depth.empty()) {
              std::cout << "Empty Image!" << std::endl;
              return 2;
          }
          cv::imshow("depth",image_depth);
          cv::imshow("color",image);
          cv::waitKey(1);

        geometry_msgs::msg::TransformStamped d435_msg;
        //vec_msg_im[i]->header.stamp = rclcpp::Clock().now();
          msg_im->header.stamp = rclcpp::Clock().now();
          msg_dp->header.stamp = rclcpp::Clock().now();
        //vec_msg_dp[i]->header.stamp = rclcpp::Clock().now();
        vec_msg_odom[i].header.stamp = rclcpp::Clock().now();
        TransformStamped_msg[i].header.stamp = rclcpp::Clock().now();
        d435_msg.header.stamp = rclcpp::Clock().now();
          cam_info_.header.stamp = rclcpp::Clock().now();
          pub_info->publish(cam_info_);

          pub_im->publish(msg_im);
          pub_dp->publish(msg_dp);

          TransformStamped_msg[i].header.frame_id = "odom_frame";
          //TransformStamped_msg[i].header.frame_id = "d435_link";
          TransformStamped_msg[i].child_frame_id = "camera_pose_optical_frame";
          br.sendTransform(TransformStamped_msg[i]);
          pub_odom->publish(vec_msg_odom[i]);

          /*socket send
          infomation.x = TransformStamped_msg[i].transform.translation.y;
          infomation.y = TransformStamped_msg[i].transform.translation.z;
          infomation.z = TransformStamped_msg[i].transform.translation.x;
          infomation.r_x = TransformStamped_msg[i].transform.rotation.y;
          infomation.r_y = TransformStamped_msg[i].transform.rotation.z;
          infomation.r_z = TransformStamped_msg[i].transform.rotation.x;
          infomation.r_w = TransformStamped_msg[i].transform.rotation.w;*/

          /* scoket注释
          infomation.x = TransformStamped_msg[i].transform.translation.y;
          infomation.y = TransformStamped_msg[i].transform.translation.z;
          infomation.z = TransformStamped_msg[i].transform.translation.x;
          infomation.r_x = TransformStamped_msg[i].transform.rotation.y;
          infomation.r_y = TransformStamped_msg[i].transform.rotation.z;
          infomation.r_z = TransformStamped_msg[i].transform.rotation.x;
          infomation.r_w = TransformStamped_msg[i].transform.rotation.w;

          memset(temp,0,sizeof(temp));         // 对.transform该内存段进行清
          memcpy(temp,&infomation,sizeof(INFOMATION));     // 把这个结构体中的信息从内存中读入到字符串temp中
          send(sock,temp,sizeof(INFOMATION) ,0);
        */
          //std::cout<<"x:"<<<<" y:"<<TransformStamped_msg[i].transform.translation.y<<" z:"<<TransformStamped_msg[i].transform.translation.z<<endl;

          d435_msg.header.frame_id = "d435_link";
          d435_msg.child_frame_id = "camera_depth_optical_frame";
          //camera_depth_optical_frame 0 -0 -0 -0.5 0.5 -0.5 0.5
          //camera_infra1_optical_frame 0 -0 -0 -0.5 0.5 -0.5 0.5
          //camera_infra2_optical_frame 0 -0.0498083 -0 -0.5 0.5 -0.5 0.5
          //camera_color_optical_frame -0.000558631 0.0146714 0.000200898 -0.5 0.5 -0.5 0.5
          d435_msg.transform.translation.x = 0;
          d435_msg.transform.translation.y = -0;
          d435_msg.transform.translation.z = -0;
          d435_msg.transform.rotation.x = -0.5;
          d435_msg.transform.rotation.y = 0.5;
          d435_msg.transform.rotation.z = -0.5;
          d435_msg.transform.rotation.w = 0.5;
          static_broadcaster->sendTransform(d435_msg);
          d435_msg.child_frame_id = "camera_infra1_optical_frame";
          static_broadcaster->sendTransform(d435_msg);
          d435_msg.child_frame_id = "camera_infra2_optical_frame";
          static_broadcaster->sendTransform(d435_msg);
          d435_msg.child_frame_id = "camera_color_optical_frame";
          static_broadcaster->sendTransform(d435_msg);


          //camera_infra1_optical_frame  camera_infra2_optical_frame  camera_color_optical_frame  camera_gyro_optical_frame  camera_accel_optical_frame
        //usleep(60000);
          //usleep(300000); //1000us是1ms
          usleep(33000); //1000us是1ms
        //cv::waitKey(60);
      }

      //for(int i = IMAGENUM - 2;i > 0;i--)
      //{
       // vec_msg_im[i]->header.stamp = rclcpp::Clock().now();
        //pub_im->publish(vec_msg_im[i]);
	    //cv::waitKey(1);
      //}
      loop_rate.sleep();
      executor.spin_once(std::chrono::seconds(0));
      //executor_dp.spin_once(std::chrono::seconds(0));
  }
  //close(sock);
  vec_msg_im.clear();
  vec_msg_dp.clear();
  return 0;
}
