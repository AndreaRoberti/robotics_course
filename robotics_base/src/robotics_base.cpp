#include <robotics_base/robotics_base.h>
RoboticsBase::RoboticsBase(ros::NodeHandle &nh) : nh_(nh),
                                                  private_nh_("~")
{
    // ntd
}

RoboticsBase::~RoboticsBase()
{
}

void RoboticsBase::init()
{
    private_nh_.param("pose_name", pose_name_, std::string("/pose_name_default"));
    private_nh_.param("angular_velocity", angular_velocity_, 0.1);
    private_nh_.param("radius", radius_, 1.0);

    pose_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>(pose_name_, 1);
}

void RoboticsBase::update()
{
    // ROS_INFO_STREAM("update");
    getTransform();

    geometry_msgs::PoseStamped pose_ee;
    pose_ee.header.frame_id = "world";
    pose_ee.header.stamp = ros::Time::now();
    pose_ee.pose.position.x = 0.0;
    pose_ee.pose.position.y = 2.0;
    pose_ee.pose.position.z = 1.0;
    pose_pub_.publish(pose_ee);
}

void RoboticsBase::getTransform()
{
    try
    {
        listener_.waitForTransform("world", "base_link", ros::Time(0), ros::Duration(1.0));
        listener_.lookupTransform("world", "base_link", ros::Time(0), base_to_world_);

        Eigen::Vector3d eigen_translation(base_to_world_.getOrigin().x(), base_to_world_.getOrigin().y(), base_to_world_.getOrigin().z());
        Eigen::Quaterniond eigen_rotation(base_to_world_.getRotation().x(), base_to_world_.getRotation().y(), base_to_world_.getRotation().z(), base_to_world_.getRotation().w());

        // Construct Eigen Affine transform
        Eigen::Affine3d eigen_transform = Eigen::Translation3d(eigen_translation) * eigen_rotation;
        Eigen::Matrix4d matrix = eigen_transform.matrix();

        // Extract rotation matrix
        Eigen::Matrix3d rotation_matrix = matrix.block<3, 3>(0, 0);

        // Convert rotation matrix to quaternion
        Eigen::Quaterniond quaternion(rotation_matrix);

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(base_to_world_.getOrigin().x() + 1.0, base_to_world_.getOrigin().y() + 0.5, 1.0));
        tf::Quaternion q;
        q.setRPY(0, 0, 1.57);
        transform.setRotation(q);
        br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "link_1"));

                // Calculate new transform
        double time_now = ros::Time::now().toSec();
        double angle = angular_velocity_ * time_now;
        double x = transform.getOrigin().x() + radius_ * std::cos(angle);
        double z = transform.getOrigin().z() + radius_ * std::sin(angle);
        tf::Transform transform_rot_xz;
        transform_rot_xz.setOrigin(tf::Vector3(x, 0, z));
        transform_rot_xz.setRotation(tf::Quaternion::getIdentity());
        br_.sendTransform(tf::StampedTransform(transform_rot_xz, ros::Time::now(), "world", "link_circular"));
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotics_base");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    RoboticsBase robotics_base(nh);
    robotics_base.init();

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        robotics_base.update();
        ros::spinOnce();
        if (!loop_rate.sleep())
        {
            ROS_WARN_STREAM("Cannot keep the update frequency");
        }
    }

    return 0;
}