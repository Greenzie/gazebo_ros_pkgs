/* Copyright [2015] [Alessandro Settimi]
 * 
 * email: ale.settimi@gmail.com
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.*/

#ifndef GAZEBO_ROS_IMU_SENSOR_H
#define GAZEBO_ROS_IMU_SENSOR_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <string>

namespace gazebo
{
  namespace sensors
  {
    class ImuSensor;
  }
  /**
  @anchor GazeboRosImuSensor
  \ref GazeboRosImuSensor is a plugin to simulate an Inertial Motion Unit sensor, the main differences from \ref GazeboRosIMU are:
  - inheritance from SensorPlugin instead of ModelPlugin,
  - measurements are given by gazebo ImuSensor instead of being computed by the ros plugin,
  - gravity is included in inertial measurements.
  */
  /** @brief Gazebo Ros imu sensor plugin. */
  class GazeboRosImuSensor : public SensorPlugin
  {
  public:
    enum DropoutSet
    {
        DROPOUT_NONE = 0,
        DROPOUT_ONCE = 1,
        DROPOUT_REPEAT = 2
    };

    /// \brief Constructor.
    GazeboRosImuSensor();
    /// \brief Destructor.
    virtual ~GazeboRosImuSensor();
    /// \brief Load the sensor.
    /// \param sensor_ pointer to the sensor.
    /// \param sdf_ pointer to the sdf config file.
    virtual void Load(sensors::SensorPtr sensor_, sdf::ElementPtr sdf_);

  protected:
    /// \brief Update the sensor.
    virtual void UpdateChild(const gazebo::common::UpdateInfo &/*_info*/);

  private:
    /// \brief Load the parameters from the sdf file.
    bool LoadParameters();
    /// \brief Gaussian noise generator.
    /// \param mu offset value.
    /// \param sigma scaling value.
    double GuassianKernel(double mu, double sigma);
    
    /// \brief Ros NodeHandle pointer.
    ros::NodeHandle* node;
    /// \brief Ros Publisher for imu data.
    ros::Publisher imu_data_publisher;
    /// \brief Ros IMU message.
    sensor_msgs::Imu imu_msg;

    /// \brief last time on which the data was published.
    common::Time last_time;
    /// \brief Pointer to the update event connection.
    gazebo::event::ConnectionPtr connection;
    /// \brief Pointer to the sensor.
    sensors::ImuSensor* sensor;
    /// \brief Pointer to the sdf config file.
    sdf::ElementPtr sdf;
    /// \brief Orientation data from the sensor.
    ignition::math::Quaterniond orientation;
    /// \brief Linear acceleration data from the sensor.
    ignition::math::Vector3d accelerometer_data;
    /// \brief Angular velocity data from the sensor.
    ignition::math::Vector3d gyroscope_data;
    /// \brief Handles dropout/delay control
    bool is_sending{true};  // Default unless in a dropout or delayed start
    double next_dropout_change_s{0.0};  // Time until next dropout change in seconds from the last change
    common::Time last_dropout_change{0.0};  // Time of the last dropout change in seconds

    //loaded parameters
    /// \brief The data is published on the topic named: /robot_namespace/topic_name.
    std::string robot_namespace;
    /// \brief The data is published on the topic named: /robot_namespace/topic_name.
    std::string topic_name;
    /// \brief Name of the link of the IMU.
    std::string body_name;
    /// \brief Sensor update rate.
    double update_rate;
    /// \brief Gaussian noise for the gyros.
    double gaussian_noise_gyro;
    /// \brief Gaussian noise for the accelerometers.
    double gaussian_noise_accelerometer;
    /// \brief Gaussian noise for the magnetometer
    double gaussian_noise_magnetometer;
    /// \brief Offset parameter, position part is unused.
    ignition::math::Pose3d offset;
    /// \brief Optional offset parameter for gyro to simulate miscalibration/bias
    ignition::math::Vector3d gyro_offset;
    /// \brief Optional delay in seconds before sending data (and before the first dropout delay)
    double delayed_start_min_s{0.0};
    double delayed_start_max_s{0.0};
    /// \brief Optional dropout parameters
    double dropout_length_min_s{0.0};
    double dropout_length_max_s{0.0};
    double dropout_delay_min_s{0.0};
    double dropout_delay_max_s{0.0};
    /// \brief Sets whether the dropout occurs (0=no, 1=once, 2=repeating)
    DropoutSet dropout_set{DropoutSet::DROPOUT_NONE};
  };
}

#endif //GAZEBO_ROS_IMU_SENSOR_H
