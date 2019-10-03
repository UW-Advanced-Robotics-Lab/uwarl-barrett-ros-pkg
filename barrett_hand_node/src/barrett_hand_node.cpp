/*
 Copyright 2019 Barrett Technology <support@barrett.com>

 This file is part of barrett-ros-pkg.

 This version of barrett-ros-pkg is free software: you can redistribute it
 and/or modify it under the terms of the GNU General Public License as
 published by the Free Software Foundation, either version 3 of the
 License, or (at your option) any later version.

 This version of barrett-ros-pkg is distributed in the hope that it will be
 useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License along
 with this version of barrett-ros-pkg.  If not, see
 <http://www.gnu.org/licenses/>.

 Barrett Technology holds all copyrights on barrett-ros-pkg. As the sole
 copyright holder, Barrett reserves the right to release future versions
 of barrett-ros-pkg under a different license.

 File: wam_node.cpp
 Date edited: 29 May, 2019
 Authors: Kyle Maroney, Alexandros Lioulemes
 */

#include <unistd.h>
#include <math.h>

#include <boost/thread.hpp> // BarrettHand threading
#include <boost/bind.hpp>

#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "wam_msgs/FtTorques.h"
#include "wam_msgs/tactilePressure.h"
#include "wam_msgs/tactilePressureArray.h"
#include "wam_srvs/BHandFingerPos.h"
#include "wam_srvs/BHandGraspPos.h"
#include "wam_srvs/BHandSpreadPos.h"
#include "wam_srvs/BHandFingerVel.h"
#include "wam_srvs/BHandGraspVel.h"
#include "wam_srvs/BHandSpreadVel.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Wrench.h"

#include <barrett/math.h> 
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems/wam.h>
#include <barrett/detail/stl_utils.h>

static const int PUBLISH_FREQ = 250; // Publishing Frequency for the BarretHand
static const double SPEED = 0.03; // Default Cartesian Velocity

using namespace barrett;
//BarrettHandNode Class
class BarrettHandNode
  {
    BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;
  protected:
    Hand* hand;
    ForceTorqueSensor* fts;
    //Subscriber
    //Published Topics
    sensor_msgs::JointState bhand_joint_state;
    wam_msgs::tactilePressureArray tactileStates;
    wam_msgs::tactilePressure tactileState;
    geometry_msgs::PoseStamped wam_pose;
    geometry_msgs::Wrench fts_state;
    wam_msgs::FtTorques ftTorque_state;
    //Publishers
    ros::Publisher bhand_joint_state_pub, fts_pub, tps_pub, fingerTs_pub;
    cf_type cf;
    ct_type ct;
    //Services
    ros::ServiceServer hand_open_grsp_srv, hand_close_grsp_srv, hand_open_sprd_srv;
    ros::ServiceServer hand_close_sprd_srv, hand_fngr_pos_srv, hand_fngr_vel_srv;
    ros::ServiceServer hand_grsp_pos_srv, hand_grsp_vel_srv, hand_sprd_pos_srv;
    ros::ServiceServer hand_sprd_vel_srv;

  public:
    BarrettHandNode() : hand(NULL)
    {
    }
    void
    init(ProductManager& pm);

    ~BarrettHandNode()
    {
    }
    bool
    handOpenGrasp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool
    handCloseGrasp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool
    handOpenSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool
    handCloseSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool
    handFingerPos(wam_srvs::BHandFingerPos::Request &req, wam_srvs::BHandFingerPos::Response &res);
    bool
    handGraspPos(wam_srvs::BHandGraspPos::Request &req, wam_srvs::BHandGraspPos::Response &res);
    bool
    handSpreadPos(wam_srvs::BHandSpreadPos::Request &req, wam_srvs::BHandSpreadPos::Response &res);
    bool
    handFingerVel(wam_srvs::BHandFingerVel::Request &req, wam_srvs::BHandFingerVel::Response &res);
    bool
    handGraspVel(wam_srvs::BHandGraspVel::Request &req, wam_srvs::BHandGraspVel::Response &res);
    bool
    handSpreadVel(wam_srvs::BHandSpreadVel::Request &req, wam_srvs::BHandSpreadVel::Response &res);
    void
    publishHand();
    void
    publishFTS();
  };

void BarrettHandNode::init(ProductManager &pm)
  {
    ros::NodeHandle nh_("bhand"); // BarrettHand specific nodehandle
    std::cout << "Barrett Hand" << std::endl;
    hand = pm.getHand();
    if (pm.foundForceTorqueSensor())
    {
      std::cout << "Force/Torque sensor" << std::endl;
      fts = pm.getForceTorqueSensor();
	    fts->tare();
	    //Publishing the following topics only if there is a BarrettHand present
      fts_pub = nh_.advertise < geometry_msgs::Wrench > ("fts_states", 1); // fts/states
    }
    usleep(500000);
    hand->initialize();
    hand->update();
    bhand_joint_state_pub = nh_.advertise < sensor_msgs::JointState > ("joint_states", 1); // bhand/joint_states
    tps_pub = nh_.advertise < wam_msgs::tactilePressureArray > ("tactile_states", 1); // tactile  sensors
    fingerTs_pub = nh_.advertise < wam_msgs::FtTorques > ("finger_tip_states", 1); // finger tip torques
    hand_open_grsp_srv = nh_.advertiseService("open_grasp", &BarrettHandNode::handOpenGrasp, this); // bhand/open_grasp
    hand_close_grsp_srv = nh_.advertiseService("close_grasp", &BarrettHandNode::handCloseGrasp, this); // bhand/close_grasp
    hand_open_sprd_srv = nh_.advertiseService("open_spread", &BarrettHandNode::handOpenSpread, this); // bhand/open_spread
    hand_close_sprd_srv = nh_.advertiseService("close_spread", &BarrettHandNode::handCloseSpread, this); // bhand/close_spread
    hand_fngr_pos_srv = nh_.advertiseService("finger_pos", &BarrettHandNode::handFingerPos, this); // bhand/finger_pos
    hand_grsp_pos_srv = nh_.advertiseService("grasp_pos", &BarrettHandNode::handGraspPos, this); // bhand/grasp_pos
    hand_sprd_pos_srv = nh_.advertiseService("spread_pos", &BarrettHandNode::handSpreadPos, this); // bhand/spread_pos
    hand_fngr_vel_srv = nh_.advertiseService("finger_vel", &BarrettHandNode::handFingerVel, this); // bhand/finger_vel
    hand_grsp_vel_srv = nh_.advertiseService("grasp_vel", &BarrettHandNode::handGraspVel, this); // bhand/grasp_vel
    hand_sprd_vel_srv = nh_.advertiseService("spread_vel", &BarrettHandNode::handSpreadVel, this); // bhand/spread_vel
    const char* bhand_jnts[] = {"inner_f1", "inner_f2", "inner_f3", "spread", "outer_f1", "outer_f2", "outer_f3"};
    std::vector < std::string > bhand_joints(bhand_jnts, bhand_jnts + 7);
    tactileState.pressure.resize(24);
    tactileState.normalizedPressure.resize(24);
    tactileStates.tactilePressures.resize(4);
    ftTorque_state.torque.resize(4);
    bhand_joint_state.name.resize(7);
    bhand_joint_state.name = bhand_joints;
    bhand_joint_state.position.resize(7);
  }

//Function to open the BarrettHand Grasp
bool BarrettHandNode::handOpenGrasp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    ROS_INFO("Opening the BarrettHand Grasp");
    hand->open(Hand::GRASP, false);
    return true;
  }

//Function to close the BarrettHand Grasp
bool BarrettHandNode::handCloseGrasp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    ROS_INFO("Closing the BarrettHand Grasp");
    hand->close(Hand::GRASP, false);
    return true;
  }

//Function to open the BarrettHand Spread
bool BarrettHandNode::handOpenSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    ROS_INFO("Opening the BarrettHand Spread");
    hand->open(Hand::SPREAD, false);
    return true;
  }

//Function to close the BarrettHand Spread
bool BarrettHandNode::handCloseSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    ROS_INFO("Closing the BarrettHand Spread");
    hand->close(Hand::SPREAD, false);
    return true;
  }

//Function to control a BarrettHand Finger Position
bool BarrettHandNode::handFingerPos(wam_srvs::BHandFingerPos::Request &req, wam_srvs::BHandFingerPos::Response &res)
  {
    ROS_INFO("Moving BarrettHand to Finger Positions: %.3f, %.3f, %.3f radians", req.radians[0], req.radians[1],
             req.radians[2]);
    hand->trapezoidalMove(Hand::jp_type(req.radians[0], req.radians[1], req.radians[2], 0.0), Hand::GRASP, false);
    return true;
  }

//Function to control the BarrettHand Grasp Position
bool BarrettHandNode::handGraspPos(wam_srvs::BHandGraspPos::Request &req, wam_srvs::BHandGraspPos::Response &res)
  {
    ROS_INFO("Moving BarrettHand Grasp: %.3f radians", req.radians);
    hand->trapezoidalMove(Hand::jp_type(req.radians), Hand::GRASP, false);
    return true;
  }

//Function to control the BarrettHand Spread Position
bool BarrettHandNode::handSpreadPos(wam_srvs::BHandSpreadPos::Request &req, wam_srvs::BHandSpreadPos::Response &res)
  {
    ROS_INFO("Moving BarrettHand Spread: %.3f radians", req.radians);
    hand->trapezoidalMove(Hand::jp_type(req.radians), Hand::SPREAD, false);
    return true;
  }

//Function to control a BarrettHand Finger Velocity
bool BarrettHandNode::handFingerVel(wam_srvs::BHandFingerVel::Request &req, wam_srvs::BHandFingerVel::Response &res)
  {
    ROS_INFO("Moving BarrettHand Finger Velocities: %.3f, %.3f, %.3f m/s", req.velocity[0], req.velocity[1],
             req.velocity[2]);
    hand->velocityMove(Hand::jv_type(req.velocity[0], req.velocity[1], req.velocity[2], 0.0), Hand::GRASP);
    return true;
  }

//Function to control a BarrettHand Grasp Velocity
bool BarrettHandNode::handGraspVel(wam_srvs::BHandGraspVel::Request &req, wam_srvs::BHandGraspVel::Response &res)
  {
    ROS_INFO("Moving BarrettHand Grasp: %.3f m/s", req.velocity);
    hand->velocityMove(Hand::jv_type(req.velocity), Hand::GRASP);
    return true;
  }

//Function to control a BarrettHand Spread Velocity
bool BarrettHandNode::handSpreadVel(wam_srvs::BHandSpreadVel::Request &req, wam_srvs::BHandSpreadVel::Response &res)
  {
    ROS_INFO("Moving BarrettHand Spread: %.3f m/s", req.velocity);
    usleep(5000);
    hand->velocityMove(Hand::jv_type(req.velocity), Hand::SPREAD);
    return true;
  }

void BarrettHandNode::publishFTS()
{
  fts->update(); // Update the hand sensors
  cf = math::saturate(fts->getForce(), 99.99);
  ct = math::saturate(fts->getTorque(), 9.999);
  // Force vector
  fts_state.force.x = cf[0];
  fts_state.force.y = cf[1];
  fts_state.force.z = cf[2];
  // Torque vector
  fts_state.torque.x = ct[0];
  fts_state.torque.y = ct[1];
  fts_state.torque.z = ct[2];
  fts_pub.publish(fts_state);
}

void BarrettHandNode::publishHand()
  {
    while (ros::ok())
    {
      hand->update(); // Update the hand sensors
      std::vector<TactilePuck*> tps = hand->getTactilePucks();
      std::vector<int> fingerTip = hand->getFingertipTorque();
      Hand::jp_type hi = hand->getInnerLinkPosition(); // get finger positions information
      Hand::jp_type ho = hand->getOuterLinkPosition();
      for (unsigned i = 0; i < tps.size(); i++)
      {
        TactilePuck::v_type pressures(tps[i]->getFullData());
        for (int j = 0; j < pressures.size(); j++) {
          int value = (int)(pressures[j] * 256.0) / 102;  // integer division
          tactileState.pressure[j] = pressures[j];
          int c = 0;
          int chunk;
          for (int z = 4; z >= 0; --z) {
            chunk = (value <= 7) ? value : 7;
            value -= chunk;
            switch (chunk)
            {
            default:
              c = c + 4;
              break;
            case 2:
              c = c + 3;
              break;
            case 1:
              c = c + 2;
              break;
            case 0:
              c = c + 1;
              break;
            }
            switch (chunk - 4) {
            case 3:
              c = c + 4;
              break;
            case 2:
              c = c+ 3;
              break;
            case 1:
              c = c + 2;
              break;
            case 0:
              c = c + 1;
              break;
            default:
              c = c + 0;
              break;
            }
          }
          tactileState.normalizedPressure[j] = c - 5;
        }
        tactileStates.tactilePressures[i] = tactileState;
      }
      for (unsigned i = 0; i < fingerTip.size(); i++)
      {
        ftTorque_state.torque[i] = fingerTip[i];
      }
      for (size_t i = 0; i < 4; i++) // Save finger positions
        bhand_joint_state.position[i] = hi[i];
      for (size_t j = 0; j < 3; j++)
        bhand_joint_state.position[j + 4] = ho[j];
      bhand_joint_state.header.stamp = ros::Time::now(); // Set the timestamp
      bhand_joint_state_pub.publish(bhand_joint_state); // Publish the BarrettHand joint states
      tps_pub.publish(tactileStates);
      fingerTs_pub.publish(ftTorque_state);
    }
  }

int main(int argc, char **argv)
{
  ProductManager pm;
  ros::init(argc, argv, "barrett_hand_node");
  BarrettHandNode barrett_hand_node;
  barrett_hand_node.init(pm);
  ros::Rate pub_rate(PUBLISH_FREQ);
  while (ros::ok())
  {
    barrett_hand_node.publishHand();
    barrett_hand_node.publishFTS();
    ros::spinOnce();
    pub_rate.sleep();
  }
  return 0;
}


