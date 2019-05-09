/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef motion_H
#define motion_H
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "eigen3/Eigen/Dense"
using namespace Eigen;
using namespace std;
class motion
{
public:
    motion();
    int main(int argc, char **argv);

private:
    void chatterCallback_base(const geometry_msgs::PoseStamped &msg);
    void chatterCallback_hand(const geometry_msgs::PoseStamped &msg);
    void chatterCallback_end(const geometry_msgs::Pose &msg);
    void Topic_initialization();
    void Init_parameters();

    ros::Subscriber basesub;
    ros::Subscriber handsub;
    ros::Subscriber endsub;
    ros::Publisher desiredpub;

    Vector3d Hand_pos;
    Vector3d base_pos;
    Vector3d end_pos;
    VectorXd end_orientation;
    Vector3d desired_pos;
    geometry_msgs::Pose desired_pose;

    bool Base_received;
    bool Hand_received;
    bool End_received;
};
#endif