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
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_msgs/msg/tf_message.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/transform.h"
#include "geometry_msgs/msg/transform.hpp"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctime>
#include <sys/time.h>
#include <string>
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
      
int image_num = 0;
const float pi = 3.1415926535898;
//单个像素点被判断为障碍物的阈值
const int threshold = 25;
//检测墙壁时，单个像素点的阈值
const int corridor_threshold = 35;
//一块block被判断为堵塞的阈值
const int block_threshold = 30;
const int block_lowerthreshold = 5;
//车辆宽度
const float car_width = 0.55;
//每次向前探测的距离
const float safety_distance = 3;
//岔路口判断距离
const float safety_turn_threshold = 2;
//转向后安全距离
const float safety_distance_after_turn = 1.5;
//向左右探测走廊的宽度
const float detection_distance = 2;
//单个block的长度
const float block_width = 0.20;
//当转弯点小于该距离时，判定需要立即转弯
const float safety_turn_distance = 0.6;

struct RPY
{
    double roll, pitch, yaw;
};

class SocketServer {
public:
    SocketServer() {
        memset((void *)&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(8000);

        l_fd = socket(AF_INET, SOCK_STREAM, 0);
        bind(l_fd, (struct sockaddr *)&server_addr, sizeof(server_addr));
        listen(l_fd, 1);

        std::cout << "waiting for connection" << std::endl;

        c_fd = accept(l_fd, (struct sockaddr *)&accept_addr, &len);
        std::cout << c_fd << std::endl;
        std::cout << "connected, ready for transmission" << std::endl;
    }

    // void initialization() {
    //     WORD w_req = MAKEWORD(2,2);
    //     WSADATA wsadata;
    //     int err;
    //     err = WSAStartup(w_req, &wsadata);
    //     if (err != 0) {
    //         std::cout << "initialization success" << std::endl;
    //     } else {
    //         std::cout << "initialization failed" << std::endl;
    //     }

    //     if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
    //         std::cout << "version incorrect" << std::endl;
    //         WSACleanup();
    //     }
    // }

    void sendMessage(const char* message, int len) {
        int send_len = write(c_fd, message, len);
        if (send_len < 0) {
            std::cout << "failed to send" << std::endl;
        }
    }

private:
    struct sockaddr_in server_addr;
    struct sockaddr_in accept_addr;
    int l_fd, c_fd;
    socklen_t len = sizeof(struct sockaddr_in);
};



class MyNode : public rclcpp::Node{
public:
    MyNode() : Node("occupancy_grid")
    {
        transform_lock = false;
        gettimeofday(&last_time, NULL);
    }

    float dis(int x, int y) {
        return sqrt(x*x+y*y)*resolution;
    }

    int min(int a, int b) {
        return a > b ? b : a;
    }

    int min3(float a, float b, float c) {
        if (a < b && a < c && a != -1) return a;
        if (b < c && b != -1) return b;
        else return c;
    }

    float linear(std::vector<float> v) {
        const int num = 6;  //num = v.size()
        int n = num/2;
        float sum = 0;
        for (int i = 0; i < n; i++) {
            if (v[i+n] < 0 || v[i] < 0)
                return 0;
            sum += v[i+n] - v[i];
        }
        sum = sum / float(n*n);
        return sum / block_width;
    }

    float least_sq(std::vector<float> v) {
        // v=ax+b
        int num = v.size();
        float A = 0, B = 0, C = 0, D = 0;
        for (int i = 0; i < num; i++) {
            A += i*i;
            B += i;
            C += i*v[i];
            D += v[i];
        }
        float a = (C*num-B*D)/(A*num-B*B);
        float b = (A*D-C*B)/(A*num-B*B);
        
        float x0 = B/num, v0 = D/num, sx = 0, sv = 0;
        for (int i = 0; i < num; i++) {
            sx += (i-x0)*(i-x0);
            sv += (v[i]-v0)*(v[i]-v0);
        }
        sx = sqrt(sx/(num-1));
        sv = sqrt(sv/(num-1));

        float r = 0;
        for (int i =0; i < num; i++) {
            r += (i-x0)/sx*(v[i]-v0)/sv/(num-1);
        }

        if (r >= 0.8) return a/0.2;
        else return 0;
    }

    void map_process() 
    {
        struct timeval time1;
        gettimeofday(&time1, NULL);

        std::cout << "map map_process" << std::endl;
        bool messageSent = false;
        orientation_x = map_to_odom.yaw + odom_to_camera.yaw;
        if (waiting) {
            if (dis(camera_cell_x-camera_cell_x0,camera_cell_y-camera_cell_y0) > 0.2) 
                waiting = 0;
            else if (abs(orientation_x-orientation_x0)/pi*180 > 10) {
                if (orientation_x > orientation_x0) {
                    //右转
                    std::string turn_message = "10" + std::to_string(90-abs(orientation_x-orientation_x0)/pi*180) + "\n";
                    // socketServer.sendMessage(m, 5);
                    // messageSent = true;
                    waiting = 0;
                    return;
                } else {
                    std::string turn_message = "00" + std::to_string(90-abs(orientation_x-orientation_x0)/pi*180) + "\n";
                    // socketServer.sendMessage(m, 5);
                    // messageSent = true;
                    waiting = 0;
                    return;
                }
            } else return;
        }
        int forward = 1;
        //todo: tan=inf时
        float tanx = tan(orientation_x);
        // std::cout << "orientation  " << orientation_x/pi*180  << " " << forward << std::endl;
        // std::cout << orientation_x << " " << -pi/2 << " " << -3*pi/2 << std::endl;

        //在图上显示小车的朝向
        // int cx, cy;
        // for (int i = 0; i < 5; i++) {
        //     cx = camera_cell_x + i*forward;
        //     cy = camera_cell_y + i*forward*tanx;
        //     color_date[cy*width+cx] = 0;
        // }
        // cv::Mat Img(height, width, CV_8UC1,color_date);
        // cv::imshow("test",Img);
        // cv::waitKey(1);

        //检测直行
        int num_block = 0, num_block_r = 0, num_block_l = 0;
        int num_block_l1 = 0, num_block_l2 = 0, num_block_r1 = 0, num_block_r2 = 0;
        int max_block = 0;
        float near_l = -1, near_r = -1;
        bool corridor_l = false, corridor_r = false;
        int danger_d = -1;
        int x = camera_cell_x, y = camera_cell_y;
        std::vector<int> blocks,blocks_r,blocks_l;
        //记录能否绕行
        std::vector<int> blocks_l1, blocks_l2, blocks_r1, blocks_r2;
        std::vector<float> nearest_l,nearest_r;

        if (abs(tanx) < 1) {
            if (cos(orientation_x) < 0) forward = -1;
            std::cout << "tanx " << tanx << std::endl;
            float distance = sqrt(1+tanx*tanx)*resolution;
            std::cout << "distance  " << distance << std::endl;
            int last_x = x;
            while (abs(x-camera_cell_x)*distance <= safety_distance) {
                x += forward;
                y = round(camera_cell_y+(x-camera_cell_x)*forward*tanx);
                if (x < 0 || y < 0 || x >= width || y >= height) {
                    std::cout << "break " << std::endl;
                    break;
                }
                //车宽
                int xx = x, yy = y, k = 0;
                while (dis(xx-x, yy-y) <= car_width/2) {
                    yy = y + k*forward;
                    xx = round(x-k*forward*tanx);
                    if (!(xx < 0 || yy < 0 || xx >= width || yy >= height)) {
                        int prob = int8_data[yy*width+xx] + 0;
                        //std::cout << prob << std::endl;
                        if (prob > threshold) {
                            num_block += 1;
                        }   
                    }

                    yy = y - k*forward;
                    xx = round(x+k*forward*tanx);
                    if (!(xx < 0 || yy < 0 || xx >= width || yy >= height)) {
                        int prob = int8_data[yy*width+xx] + 0;
                        //std::cout << prob << std::endl;
                        if (prob > threshold) {
                            //std::cout << prob << std::endl;
                            num_block += 1;
                        }
                    }
                    k += 1;
                }

                //转向
                while (dis(xx-x, yy-y) <= car_width/2 + safety_distance_after_turn) {
                    yy = y + k*forward;
                    xx = round(x-k*forward*tanx);
                    if (!(xx < 0 || yy < 0 || xx >= width || yy >= height)) {
                        int prob = int8_data[yy*width+xx] + 0;
                        //std::cout << prob << std::endl;
                        if (prob > threshold) {
                            //std::cout << prob << std::endl;
                            if (dis(xx-x,yy-y) <= car_width/2 + car_width) {
                                num_block_l1 += 1;
                            } else if (dis(xx-x,yy-y) <= car_width/2 + car_width*2) {
                                num_block_l2 += 1;
                            }
                            num_block_l += 1;
                            if (prob > corridor_threshold && near_l < 0 && dis(xx-x, yy-y) > 0)
                                near_l = dis(xx-x, yy-y);
                        }
                    } else num_block_l = -1;
                

                    yy = y - k*forward;
                    xx = round(x+k*forward*tanx);
                    if (!(xx < 0 || yy < 0 || xx >= width || yy >= height)) {
                        int prob = int8_data[yy*width+xx] + 0;
                        //std::cout << prob << std::endl;
                        if (prob > threshold) {
                            if (dis(xx-x,yy-y) <= car_width/2 + car_width) {
                                num_block_r1 += 1;
                            } else if (dis(xx-x,yy-y) <= car_width/2 + car_width*2) {
                                num_block_r2 += 1;
                            }
                            num_block_r += 1;
                            if (prob > corridor_threshold && near_r < 0 && dis(xx-x, yy-y) > 0)
                                near_r = dis(xx-x, yy-y);
                        }   
                    } else num_block_r = -1;
                
                    k += 1;
                }

                //走廊
                while (dis(xx-x, yy-y) <= car_width/2 + detection_distance && (near_l < 0 || near_r < 0)) {
                    yy = y + k*forward;
                    xx = round(x-k*forward*tanx);
                    if (!(xx < 0 || yy < 0 || xx >= width || yy >= height)) {
                        int prob = int8_data[yy*width+xx] + 0;
                        if (prob > corridor_threshold) {
                            if (near_l < 0)
                                near_l = dis(xx-x, yy-y);
                        }
                    }
                

                    yy = y - k*forward;
                    xx = round(x+k*forward*tanx);
                    if (!(xx < 0 || yy < 0 || xx >= width || yy >= height)) {
                        int prob = int8_data[yy*width+xx] + 0;
                        if (prob > corridor_threshold) {
                            if (near_r < 0)
                                near_r = dis(xx-x, yy-y);
                        }   
                    }
        
                    k += 1;
                }

                if (abs(x-last_x)*distance >= block_width) {
                    //std::cout << "actual block_width:  " << abs(x-last_x)*distance << std::endl;
                    blocks.push_back(num_block);
                    blocks_r.push_back(num_block_r);
                    blocks_l.push_back(num_block_l);
                    blocks_l1.push_back(num_block_l1);
                    blocks_l2.push_back(num_block_l2);
                    blocks_r1.push_back(num_block_r1);
                    blocks_r2.push_back(num_block_r2);
                    nearest_r.push_back(near_r);
                    nearest_l.push_back(near_l);
                    if (num_block >= block_threshold && danger_d == -1) {
                        danger_d = blocks.size();
                    }
                    num_block = 0;
                    num_block_l = 0;
                    num_block_r = 0;
                    num_block_l1 = 0;
                    num_block_l2 = 0;
                    num_block_r1 = 0;
                    num_block_r2 = 0;
                    near_l = -1;
                    near_r = -1;
                    last_x = x;
                }
            }

            if (last_x != x) {
                blocks.push_back(num_block);
                blocks_r.push_back(num_block_r);
                blocks_l.push_back(num_block_l);
                blocks_l1.push_back(num_block_l1);
                blocks_l2.push_back(num_block_l2);
                blocks_r1.push_back(num_block_r1);
                blocks_r2.push_back(num_block_r2);
                nearest_r.push_back(near_r);
                nearest_l.push_back(near_l);
                if (num_block >= block_threshold && danger_d == -1) {
                    danger_d = blocks.size();
                }
                num_block = 0;
                num_block_l = 0;
                num_block_r = 0;
                num_block_l1 = 0;
                num_block_l2 = 0;
                num_block_r1 = 0;
                num_block_r2 = 0;
                near_l = -1;
                near_r = -1;
                last_x = x;
            }
        } else {
            if (sin(orientation_x) < 0) forward = -1;
            float tany = 1/tanx;
            std::cout << "tany " << tany << std::endl;
            float distance = sqrt(1+tany*tany)*resolution;
            std::cout << "distance  " << distance << std::endl;
            int last_y = y;
            while (abs(y-camera_cell_y)*distance <= safety_distance) {
                y += forward;
                x = round(camera_cell_x+(y-camera_cell_y)*forward*tany);
                if (x < 0 || y < 0 || x >= width || y >= height) {
                    std::cout << "break " << std::endl;
                    break;
                }
                //车宽
                int xx = x, yy = y, k = 0;
                while (dis(xx-x, yy-y) <= car_width/2) {
                    xx = x + k*forward;
                    yy = round(y-k*forward*tany);
                    if (!(xx < 0 || yy < 0 || xx >= width || yy >= height)) {
                        int prob = int8_data[yy*width+xx] + 0;
                        //std::cout << prob << std::endl;
                        if (prob > threshold) {
                            num_block += 1;
                        }   
                    }

                    xx = x - k*forward;
                    yy = round(y+k*forward*tany);
                    if (!(xx < 0 || yy < 0 || xx >= width || yy >= height)) {
                        int prob = int8_data[yy*width+xx] + 0;
                        //std::cout << prob << std::endl;
                        if (prob > threshold) {
                            //std::cout << prob << std::endl;
                            num_block += 1;
                        }
                    }
                    k += 1;
                }

                //转向
                while (dis(xx-x, yy-y) <= car_width/2 + safety_distance_after_turn) {
                    xx = x + k*forward;
                    yy = round(y-k*forward*tany);
                    if (!(xx < 0 || yy < 0 || xx >= width || yy >= height)) {
                        int prob = int8_data[yy*width+xx] + 0;
                        //std::cout << prob << std::endl;
                        if (prob > threshold) {
                            //std::cout << prob << std::endl;
                            if (dis(xx-x,yy-y) <= car_width/2 + car_width) {
                                num_block_r1 += 1;
                            } else if (dis(xx-x,yy-y) <= car_width/2 + car_width*2) {
                                num_block_r2 += 1;
                            }
                            num_block_r += 1;
                            if (prob > corridor_threshold && near_r < 0 && dis(xx-x, yy-y) > 0)
                                near_r = dis(xx-x, yy-y);
                        }
                    } else num_block_r = -1;
                

                    xx = x - k*forward;
                    yy = round(y+k*forward*tany);
                    if (!(xx < 0 || yy < 0 || xx >= width || yy >= height)) {
                        int prob = int8_data[yy*width+xx] + 0;
                        //std::cout << prob << std::endl;
                        if (prob > threshold) {
                            if (dis(xx-x,yy-y) <= car_width/2 + car_width) {
                                num_block_l1 += 1;
                            } else if (dis(xx-x,yy-y) <= car_width/2 + car_width*2) {
                                num_block_l2 += 1;
                            }
                            num_block_l += 1;
                            if (prob > corridor_threshold && near_l < 0 && dis(xx-x, yy-y) > 0)
                                near_l = dis(xx-x, yy-y);
                        }   
                    } else num_block_l = -1;
                
                    k += 1;
                }

                //走廊
                while (dis(xx-x, yy-y) <= car_width/2 + detection_distance && ((near_l < 0 && corridor_l) || (near_r < 0 && corridor_r))) {
                    xx = x + k*forward;
                    yy = round(y-k*forward*tany);
                    if (!(xx < 0 || yy < 0 || xx >= width || yy >= height)) {
                        int prob = int8_data[yy*width+xx] + 0;
                        if (prob > corridor_threshold) {
                            if (near_r < 0)
                                near_r = dis(xx-x, yy-y);
                        }
                    }
                

                    xx = x - k*forward;
                    yy = round(y+k*forward*tany);
                    if (!(xx < 0 || yy < 0 || xx >= width || yy >= height)) {
                        int prob = int8_data[yy*width+xx] + 0;
                        if (prob > corridor_threshold) {
                            if (near_l < 0)
                                near_l = dis(xx-x, yy-y);
                        }   
                    }
        
                    k += 1;
                }

                if (abs(y-last_y)*distance >= block_width) {          
                    //std::cout << "actual block_width:  " << abs(y-last_y)*distance << std::endl;             
                    blocks.push_back(num_block);
                    blocks_r.push_back(num_block_r);
                    blocks_l.push_back(num_block_l);
                    blocks_l1.push_back(num_block_l1);
                    blocks_l2.push_back(num_block_l2);
                    blocks_r1.push_back(num_block_r1);
                    blocks_r2.push_back(num_block_r2);
                    nearest_r.push_back(near_r);
                    nearest_l.push_back(near_l);
                    if (num_block >= block_threshold && danger_d == -1) {
                        danger_d = blocks.size();
                    }
                    num_block = 0;
                    num_block_l = 0;
                    num_block_r = 0;
                    num_block_l1 = 0;
                    num_block_l2 = 0;
                    num_block_r1 = 0;
                    num_block_r2 = 0;
                    near_l = -1;
                    near_r = -1;
                    last_y = y;
                }
            }

            if (last_y != y) {
                blocks.push_back(num_block);
                blocks_r.push_back(num_block_r);
                blocks_l.push_back(num_block_l);
                blocks_l1.push_back(num_block_l1);
                blocks_l2.push_back(num_block_l2);
                blocks_r1.push_back(num_block_r1);
                blocks_r2.push_back(num_block_r2);
                nearest_r.push_back(near_r);
                nearest_l.push_back(near_l);
                if (num_block >= block_threshold && danger_d == -1) {
                    danger_d = blocks.size();
                }
                num_block = 0;
                num_block_l = 0;
                num_block_r = 0;
                num_block_l1 = 0;
                num_block_l2 = 0;
                num_block_r1 = 0;
                num_block_r2 = 0;
                near_l = -1;
                near_r = -1;
                last_y = y;
            }
        }

        float near_r0 = min3(nearest_r[0], nearest_r[1], nearest_r[2]);
        float near_l0 = min3(nearest_l[0], nearest_l[1], nearest_l[2]);
        std::string stop_message = "0";
        if ((near_r0 >= safety_turn_threshold || near_r0 == -1) && (near_r00 < safety_turn_threshold && near_r00 != -1))
            stop_message = "1" + stop_message;
        else stop_message = "0" + stop_message;
        if ((near_l0 >= safety_turn_threshold || near_l0 == -1) && (near_l00 < safety_turn_threshold && near_l00 != -1))
            stop_message = "1" + stop_message;
        else stop_message = "0" + stop_message;

        if (stop_message[0] == '1' || stop_message[1] == '1') {
            stop_message = "2" + stop_message + "\n";
            // socketServer.sendMessage(m, 5);
            // messageSent = true;
            near_r00 = near_r0;
            near_l00 = near_l0;
            waiting = 1;
            return;
        }
        near_r00 = near_r0;
        near_l00 = near_l0;


        float tank = 0;
        int theta = 0;
        std::string corridor_message;
        tank = least_sq(nearest_l);
        std::cout << "tank   " << tank << std::endl;
        if (tank != 0) corridor_l = true;
        else {
            tank = least_sq(nearest_r);
            if (tank != 0) corridor_r = true;
            std::cout << "tank   " << tank << std::endl;
        }
        
        if ((corridor_l || corridor_r) && danger_d == -1) {
            //send message tank
            theta = round(atan(tank)/pi*180);
            if (abs(theta) >= 100) corridor_message = std::to_string(abs(theta));
            else if (abs(theta) >= 10) corridor_message = "0"+std::to_string(abs(theta));
            else corridor_message = "00"+std::to_string(theta);

            //首位为0: 左转； 1：右转
            if (corridor_l) {    
                if (theta < 0) {
                    corridor_message = "1"+corridor_message;
                } else {
                    corridor_message = "0"+corridor_message;
                }
            } else if (corridor_r) {
                if (theta < 0) {
                    corridor_message = "0"+corridor_message;
                } else {
                    corridor_message = "1"+corridor_message;
                }
            }
            corridor_message += "\n";

            if (abs(theta) > 15 || status == 2) {
                const char* m = corridor_message.data();
                // socketServer.sendMessage(m, 5);
                // messageSent = true;
                // std::cout << "*******find corridor: " << theta << std::endl;
                // return;
                status = 1;
            } else if (abs(theta) <= 15) status = 1;
        } else status = 0;

        num_block_r = 0, num_block_l = 0;
        bool initialized = false;
        int l_min = -1, l_d = -1, r_min = -1, r_d = -1, turn_d = -1, turn = -1;
        for (int i = 0; i < blocks.size()-car_width/block_width+1; i++) {
            if (danger_d > 0 && i >= danger_d)
                break;

            if (i == 0 || !initialized) {
                for (int j = 0; j < car_width/block_width; j++){
                    if (blocks_r[i+j] == -1) {
                        num_block_r = -1;
                        break;
                    } else num_block_r += blocks_r[i+j];
                }
                for (int j = 0; j < car_width/block_width; j++){
                    if (blocks_l[i+j] == -1) {
                        num_block_l = -1;
                        break;
                    } else num_block_l += blocks_l[i+j];
                }
                if (num_block_l >= 0 && num_block_r >= 0)
                    initialized = true;
            } else {
                if (blocks_r[i+car_width/block_width-1] >= 0) 
                    num_block_r = num_block_r - blocks_r[i-1] + blocks_r[i+car_width/block_width-1];
                else {
                    num_block_r = -1;
                    initialized = false;
                }
                if (blocks_l[i+car_width/block_width-1] >= 0) 
                    num_block_l = num_block_l - blocks_l[i-1] + blocks_l[i+car_width/block_width-1];
                else {
                    num_block_l = -1;
                    initialized = false;
                }
            }

            if (num_block_r < r_min || r_min == -1) {
                r_min = num_block_r;
                r_d = i;
            }
            if (num_block_l < l_min || l_min == -1) {
                l_min = num_block_l;
                l_d = i;
            }
        }

        bool left_turn = false, right_turn = false;
        if (l_min < block_threshold && l_min >= 0) left_turn = true;
        if (r_min < block_threshold && r_min >= 0) right_turn = true;
        std::cout << "left turn: " << left_turn  << "   " << l_d << "  right turn: " << right_turn << "    " << r_d << std::endl;
        std::cout << "left block: " << l_min << "  right block: " << r_min << std::endl;

        if (avoiding == 1 && left_turn) {
            turn = 1;
            turn_d = l_d;
            avoiding = 0;
        } else if (avoiding == 2 && right_turn) {
            turn = 2;
            turn_d = r_d;
            avoiding = 0;
        } else if (danger_d != -1) {
            int turn = 0, turn_d = 0;
            if (left_turn && !right_turn) {
                turn = 1;
                turn_d = l_d;
            } else if (right_turn && !left_turn) {
                turn = 2;
                turn_d = r_d;
            } else if (!right_turn && !left_turn) {
                turn = -1;
            } else {
                int l_avoid = min(blocks_l1[danger_d],blocks_l2[danger_d]);
                int r_avoid = min(blocks_r1[danger_d],blocks_r2[danger_d]);
                if (l_avoid <= r_avoid && l_avoid < block_threshold) {
                    turn = 1;
                    turn_d = l_d;
                    avoiding = 2;
                } else if (r_avoid < l_avoid && r_avoid < block_threshold) {
                    turn = 2;
                    turn_d = r_d;
                    avoiding = 1;
                }
                if (l_min < r_min || (l_min == r_min && l_d < r_d)) {
                    turn = 1;
                    turn_d = l_d;
                } else {
                    turn = 2;
                    turn_d = r_d;
                }
            }
            //turn && status
            std::cout << l_d << " " << r_d << std::endl;
            if (turn_d*block_width <= safety_turn_distance) {
                if (turn == 1) {
                    char* message = "0090\n";
                    std::cout << "send message: " << message << std::endl;
                    // socketServer.sendMessage(message, 5);
                    messageSent = true;
                } else if (turn == 2) {
                    char* message = "1090\n";
                     std::cout << "send message: " << message << std::endl;
                    // socketServer.sendMessage(message, 5);
                    messageSent = true;
                } else {
                    //无法转向
                    std::cout << "**************调头***************" << std::endl;
                }
                status = 2;
            }
        } else if (left_turn || right_turn) {
            //直行中发现岔路口
            std::cout << "*****************found turn point********************" << std::endl;
        }

        // std::cout << "******************" << std::endl;
        // std::cout << "block_size: " << blocks.size() << std::endl;
        // for (int i = 0; i < blocks.size(); i++) 
        //     std::cout << blocks_l[i] << " " << blocks[i] << " " << blocks_r[i] << std::endl;
        // std::cout << "******************" << std::endl;

        //debug
        // if (!messageSent) {
        //     char* message = "0090\n";
        //     socketServer.sendMessage(message,5);
        // }

        //std::cout << "max_block  " << max_block << std::endl;

        if (!messageSent || true) {
            if (abs(theta) >= 15 || status == 2) {
                const char* m = corridor_message.data();
                // socketServer.sendMessage(m, 5);
                messageSent = true;
                status = 1;
            } else if (abs(theta) <= 15) status = 1;
        }
        std::cout << "*******find corridor: " << theta << std::endl;
        std::cout << "corridor_l " << corridor_l << " corridor_r " << corridor_r << "resolution  " << resolution << std::endl;
        for (int i = 0; i < nearest_l.size(); i++) 
            std::cout << nearest_l[i] << "  " << nearest_r[i] << std::endl;

        struct timeval time2;
        gettimeofday(&time2, NULL);
        int ms = (time1.tv_sec-time2.tv_sec)*1000+(time1.tv_usec-time2.tv_usec)/1000;
        std::cout << "*****************use_time: " << ms << "   ***********************" << std::endl;
    }

    void callback(const nav_msgs::msg::OccupancyGrid::SharedPtr occupancygrid)
    {
        //std::cout << "map success" << std::endl;
        height = occupancygrid->info.height;
        width = occupancygrid->info.width;
        resolution = occupancygrid->info.resolution;
        std::vector<int8_t> data = occupancygrid->data;
        int8_data = new int8_t[data.size()];
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
        //std::cout<<width<<" "<<height<<" "<<int(*min)<<" "<<int(*max)<<" "<< occupancygrid->info.origin.position.x<< "," \
        //<< occupancygrid->info.origin.position.y<< "," << occupancygrid->info.origin.position.z<< "\n";

        auto map_origin_x = -occupancygrid->info.origin.position.x;
        auto map_origin_y = -occupancygrid->info.origin.position.y;

        map_cell_x = round(map_origin_x/resolution);
        map_cell_y = round(map_origin_y/resolution);

        transform_lock = true;
        float camera_x = map_origin_x + map_to_odom_frame.translation.x + odom_frame_to_camera.translation.x;
        float camera_y = map_origin_y + map_to_odom_frame.translation.y + odom_frame_to_camera.translation.y;
        tf2::Quaternion quat_1(map_to_odom_frame.rotation.x, map_to_odom_frame.rotation.y, map_to_odom_frame.rotation.z, map_to_odom_frame.rotation.w);
        tf2::Quaternion quat_2(odom_frame_to_camera.rotation.x, odom_frame_to_camera.rotation.y, odom_frame_to_camera.rotation.z, odom_frame_to_camera.rotation.w);
        tf2::Matrix3x3(quat_1).getRPY(map_to_odom.roll, map_to_odom.pitch, map_to_odom.yaw);
        tf2::Matrix3x3(quat_2).getRPY(odom_to_camera.roll, odom_to_camera.pitch, odom_to_camera.yaw);
        transform_lock = false;

        // std::cout << map_to_odom.roll << " " << map_to_odom.pitch << " " << map_to_odom.yaw << std::endl;
        // std::cout << odom_to_camera.roll << " " << odom_to_camera.pitch << " " << odom_to_camera.yaw << std::endl;
        // std::cout << std::endl;

        camera_cell_x = round(camera_x/resolution);
        camera_cell_y = round(camera_y/resolution);

        //标记起始点和目前所在位置
        for (int i = 0; i < 3; i++) {
            color_date[map_cell_y*width+map_cell_x-i] = 0;
            color_date[map_cell_y*width+map_cell_x+i] = 0;
            color_date[map_cell_y*width+map_cell_x-i*width] = 0;
            color_date[map_cell_y*width+map_cell_x+i*width] = 0;
            color_date[camera_cell_y*width+camera_cell_x-i] = 0;
            color_date[camera_cell_y*width+camera_cell_x+i] = 0;
            color_date[camera_cell_y*width+camera_cell_x-i*width] = 0;
            color_date[camera_cell_y*width+camera_cell_x+i*width] = 0;
        }

        cv::Mat Img(height, width, CV_8UC1,color_date);
        cv::imshow("test",Img);
        cv::waitKey(1);

        struct timeval time0;
        gettimeofday(&time0, NULL);
        //map_process();
        if ((time0.tv_sec-last_time.tv_sec)*1000+(time0.tv_usec-last_time.tv_usec)/1000 > 1500) {
            last_time = time0;
            map_process();
            camera_cell_x0 = camera_cell_x;
            camera_cell_y0 = camera_cell_y;
            orientation_x0 = orientation_x;
        }
    }

    void callback_tf(const tf2_msgs::msg::TFMessage::SharedPtr message)
    {
        // std::cout<<"tf success"<<std::endl;
        // for (int i = 0; i < message->transforms.size(); i++)
        //     std::cout<<message->transforms[i].header.frame_id<<"  "<<message->transforms[i].child_frame_id<<std::endl;
        auto frame_id = message->transforms[0].header.frame_id;
        auto child_frame_id = message->transforms[0].child_frame_id;
        if ((frame_id == "map") && (child_frame_id == "odom_frame")) {
            if (!transform_lock) {
                map_to_odom_frame = message->transforms[0].transform;
            }
            //std::cout<<message->transforms[0].header.frame_id<<"  "<<message->transforms[0].child_frame_id<<std::endl;
        }
        if ((frame_id == "odom_frame") && (child_frame_id == "camera_pose_optical_frame")) {
            if (!transform_lock) {
                odom_frame_to_camera = message->transforms[0].transform;
            }
            //std::cout<<message->transforms[0].header.frame_id<<"  "<<message->transforms[0].child_frame_id<<std::endl;
        }
    }

    void callback_tf_static(const tf2_msgs::msg::TFMessage::SharedPtr message)
    {
        // std::cout<<"tf success"<<std::endl;
        // for (int i = 0; i < message->transforms.size(); i++)
        //     std::cout<<message->transforms[i].header.frame_id<<"  "<<message->transforms[i].child_frame_id<<std::endl;
    }

    void callback_odom(const nav_msgs::msg::Odometry::SharedPtr odometry)
    {
        //std::cout<<"pose success"<<std::endl;
        //std::cout<<odometry->header.frame_id<<std::endl;
        // std::cout<<odometry->pose.pose.position.x<<","<<odometry->pose.pose.position.y<<","<<odometry->pose.pose.position.z<<std::endl;
        // std::cout<<odometry->pose.pose.orientation.x<<","<<odometry->pose.pose.orientation.y<<","<<odometry->pose.pose.orientation.z<<","<<odometry->pose.pose.orientation.w<<std::endl;
    }
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscriber;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_subscriber;

    geometry_msgs::msg::Transform map_to_odom_frame, odom_frame_to_camera;
    float resolution;

private:
    int height, width;
    int map_cell_x, map_cell_y, camera_cell_x, camera_cell_y, camera_cell_x0, camera_cell_y0;
    bool transform_lock;
    int8_t *int8_data;
    RPY map_to_odom, odom_to_camera; 
    //SocketServer socketServer;
    struct timeval last_time;
    int status = 0;
    //1表示正在绕行，要向左转，2表示要向右转
    int avoiding = 0;  
    //记录左右最近的点
    float near_r00 = -2, near_l00 = -2;
    //自主选择模式 0表示不处于该模式，1表示处于该模式
    bool chose_mode = 1, waiting = 0;
    double orientation_x, orientation_x0;
    //rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscriber;
};

int main (int argc, char** argv)
{ 
    // char* message = "hello \n";
    // SocketServer socketServer;
    // socketServer.sendMessage(message, 7);
    rclcpp::init(argc, argv);
    auto node_occupancy_grid = std::make_shared<MyNode>();
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    node_occupancy_grid->subscriber = node_occupancy_grid->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&MyNode::callback, node_occupancy_grid, std::placeholders::_1));
    //node_occupancy_grid->odom_subscriber = node_occupancy_grid->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&MyNode::callback_odom, node_occupancy_grid, std::placeholders::_1));
    node_occupancy_grid->tf_subscriber = node_occupancy_grid->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 10, std::bind(&MyNode::callback_tf, node_occupancy_grid, std::placeholders::_1));
    //node_occupancy_grid->tf_static_subscriber = node_occupancy_grid->create_subscription<tf2_msgs::msg::TFMessage>("/tf_static", 10, std::bind(&MyNode::callback_tf_static, node_occupancy_grid, std::placeholders::_1));
    rclcpp::spin(node_occupancy_grid);
    rclcpp::shutdown();
    // node_occupancy_grid->pointcloud2_subscriber = node_occupancy_grid->create_subscription<sensor_msgs::msg::PointCloud2>("/scan_matched_points2", qos, std::bind(&MyNode::callback_pointcloud2, node_occupancy_grid, std::placeholders::_1));
    // node_occupancy_grid->odom_subscriber = node_occupancy_grid->create_subscription<nav_msgs::msg::Odometry>("/odom", qos, std::bind(&MyNode::callback_odom, node_occupancy_grid, std::placeholders::_1));
    // auto node = std::make_shared<nav2_costmap_2d::Costmap2DROS>("costmap");
    // rclcpp::spin(node->get_node_base_interface());
    // rclcpp::spin(node_occupancy_grid);
    // rclcpp::shutdown();
    /*
  auto node = rclcpp::Node::make_shared("realsene_image_raw_sub");
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_im = node->create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw", qos, imCallBack);
  rclcpp::WallRate loop_rate(1);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
    */
  //   rclcpp::WallRate loop_rate(1);
  //   rclcpp::executors::SingleThreadedExecutor executor;
  //   executor.add_node(node_occupancy_grid);
  // while(rclcpp::ok())
  // {
  //   loop_rate.sleep();
  //   executor.spin_once(std::chrono::seconds(0));
  // }
    
  return 0;
}  

