/*
 * Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/

/*
 * \file  gazebo_ros_diff_drive.h
 *
 * \brief A differential drive plugin for gazebo. Based on the diffdrive plugin
 * developed for the erratic robot (see copyright notice above). The original
 * plugin can be found in the ROS package gazebo_erratic_plugins.
 *
 * \author  Piyush Khandelwal (piyushk@gmail.com)
 *
 * $ Id: 06/21/2013 11:23:40 AM piyushk $
 */

#ifndef DIFFDRIVE_PLUGIN_HH
#define DIFFDRIVE_PLUGIN_HH

#include <map>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

  class Joint;
  class Entity;

  class GazeboRosDiffDrive : public ModelPlugin {

    enum OdomSource
    {
        ENCODER = 0,
        WORLD = 1,
    };

    enum DropoutSet
    {
        DROPOUT_NONE = 0,
        DROPOUT_ONCE = 1,
        DROPOUT_REPEAT = 2
    };

    enum DropoutWheel
    {
        WHEEL_TOGETHER = 0,
        WHEEL_SEPARATE = 1,
        WHEEL_LEFT = 2,
        WHEEL_RIGHT = 3,
    };

    public:
      GazeboRosDiffDrive();
      ~GazeboRosDiffDrive();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      void Reset();

    protected:
      virtual void UpdateChild();
      virtual void FiniChild();

    private:
      void publishOdometry(double step_time);
      void getWheelVelocities();
      void publishWheelTF(); /// publishes the wheel tf's
      void publishWheelJointState();
      void UpdateOdometryEncoder();


      GazeboRosPtr gazebo_ros_;
      physics::ModelPtr parent;
      event::ConnectionPtr update_connection_;

      double wheel_separation_;
      double wheel_diameter_;
      double wheel_torque;
      double wheel_speed_[2];
      double wheel_accel;
      double wheel_speed_instr_[2];

      std::vector<physics::JointPtr> joints_;

      // ROS STUFF
      ros::Publisher odometry_publisher_;
      ros::Subscriber cmd_vel_subscriber_;
      boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
      sensor_msgs::JointState joint_state_;
      ros::Publisher joint_state_publisher_;
      nav_msgs::Odometry odom_;
      std::string tf_prefix_;

      boost::mutex lock;

      std::string robot_namespace_;
      std::string command_topic_;
      std::string odometry_topic_;
      std::string odometry_frame_;
      std::string robot_base_frame_;
      bool publish_tf_;

      /// \brief Optional delay in seconds before sending data (and before the first dropout delay)
      double delayed_start_min_s_{0.0};
      double delayed_start_max_s_{0.0};
      /// \brief Optional dropout parameters
      double dropout_length_min_s_{0.0};
      double dropout_length_max_s_{0.0};
      double dropout_delay_min_s_{0.0};
      double dropout_delay_max_s_{0.0};
      /// \brief Sets whether the dropout occurs (0=no, 1=once, 2=repeating)
      DropoutSet dropout_set_{DropoutSet::DROPOUT_NONE};
      /// \brief Sets which wheel(s) dropout
      DropoutWheel dropout_wheel_{DropoutWheel::WHEEL_TOGETHER};
      /// \brief Used to simulate a bad reset, such that the encoder looks like it jumped
      /// Defined as v * ( wheel_diameter_ / 2.0 );
      double initial_jump_{0.0};

      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

      // DiffDrive stuff
      void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

      double x_;
      double rot_;
      bool alive_;

      // Update Rate
      double update_rate_;
      double update_period_;
      common::Time last_update_time_;

      /// \brief Handles dropout/delay control
      bool is_reading_{true};  // Default unless in a dropout or delayed start
      bool is_delayed_start_{false};  // Prevent sending initially
      bool last_delayed_start_{false};  // For an immediate send
      double next_dropout_change_s_{0.0};  // Time until next dropout change in seconds from the last change
      common::Time last_dropout_change_{0.0};  // Time of the last dropout change in seconds
      bool left_wheel_dropped_{false};  // Whether the left wheel has dropped out
      bool right_wheel_dropped_{false};  // Whether the right wheel has dropped out

      OdomSource odom_source_;
      geometry_msgs::Pose2D pose_encoder_;
      common::Time last_odom_update_;

    // Flags
    bool publishWheelTF_;
    bool publishOdomTF_;
    bool publishWheelJointState_;

  };

}

#endif
