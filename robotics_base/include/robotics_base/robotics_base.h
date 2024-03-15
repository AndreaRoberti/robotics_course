#pragma once

// Including necessary ROS libraries
#include <geometry_msgs/PoseStamped.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// Including Eigen libraries for linear algebra operations
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

/**
 * @brief A base class for robotics functionality.
 * 
 * This class provides a base for implementing robotics functionalities in ROS.
 */
class RoboticsBase
{
private:
    ros::NodeHandle nh_; /**< ROS NodeHandle for general use */
    ros::NodeHandle private_nh_; /**< ROS NodeHandle for private use */

    std::string pose_name_; /**< Name of the pose */

    ros::Publisher pose_pub_; /**< ROS Publisher for publishing pose */
    tf::TransformListener listener_; /**< Listener for transforming frames */
    tf::TransformBroadcaster br_; /**< Broadcaster for transforming frames */
    tf::StampedTransform base_to_world_; /**< Stamped transform from base to world frame */

    /**
     * @brief Retrieve the transform from base to world frame.
     * 
     * This function retrieves the transform from base to world frame and stores it internally.
     */
    void getTransform();

protected:
    /**
     * @brief Get the ROS NodeHandle.
     * 
     * @return Reference to the ROS NodeHandle.
     */
    virtual ros::NodeHandle &getNodeHandle()
    {
        return nh_;
    }

    /**
     * @brief Get the private ROS NodeHandle.
     * 
     * @return Reference to the private ROS NodeHandle.
     */
    virtual ros::NodeHandle &getPrivateNodeHandle()
    {
        return private_nh_;
    }

public:
    /**
     * @brief Constructs a new RoboticsBase object.
     * 
     * @param nh ROS NodeHandle reference for initializing the node.
     */
    RoboticsBase(ros::NodeHandle &nh);

    /**
     * @brief Destroys the RoboticsBase object.
     */
    ~RoboticsBase();

    /**
     * @brief Initialize the robotics base functionality.
     * 
     * This function initializes necessary components for robotics base functionality.
     */
    void init();

    /**
     * @brief Update the robotics base functionality.
     * 
     * This function updates the robotics base functionality.
     */
    void update();
};
