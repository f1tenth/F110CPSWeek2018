// ZED includes
#include <sl/Camera.hpp>
//ROS includes
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace sl;             // ZED namespace

// create ZED objects
sl::Camera          zed;
sl::Pose            camera_pose;
float               translation_left_to_center;

// ROS parameters
std::string         odom_topic;
bool                publish_tf;
std::string         base_frame_id;
std::string         odom_frame_id;
std::string         zed_frame_id;
bool                enable_spatial_memory;
float               lin_speed_x;
float               lin_speed_y;
ros::Time           old_time;


// Functions
int init(ros::NodeHandle node_handle);
tf2::Transform updatePose(ros::Time time, tf2_ros::Buffer *tfBuffer);
void publish_trasform(geometry_msgs::Transform robot_pose, tf2_ros::TransformBroadcaster  tf_broadcaster, ros::Time time);
void publish_odometry(geometry_msgs::Transform robot_pose, ros::Publisher odom_publisher, ros::Time time);
void transformPose(sl::Transform &pose, float tx);

//------------------------------------------------------------------------------
// Main function of the program, grabs parameter from launch file (optional)
// and start ZED callback function.
//------------------------------------------------------------------------------
int
main(int argc, char **argv) {
    // initialize ros node
    ros::init(argc, argv, "visual_odometry");

    // ROS objects
    ros::NodeHandle                 pn("~");
    ros::Publisher                  odom_pub;
    ros::Rate                       loop_rate(60);
    tf2_ros::TransformBroadcaster   odom_broadcaster;
    tf2_ros::Buffer                 tfBuffer;
    tf2_ros::TransformListener      tfListener(tfBuffer);
    tf2::Transform                  robot_pose;
    geometry_msgs::Transform        m_robot_pose;
    geometry_msgs::Transform        m_old_robot_pose;


    // initialize variable
    int err     = init(pn);
    if(err) {
        printf("[ERROR] During initilization of parameter.\n");
        return 0;
    }
    odom_pub = pn.advertise<nav_msgs::Odometry>(odom_topic, 5);
    // Get the distance between the center of the camera and the left eye
    translation_left_to_center = zed.getCameraInformation().calibration_parameters.T.x * 0.5f;

    // main loop
    while(ros::ok()) {
        if (zed.grab() == SUCCESS) {
            ros::Time t     = ros::Time::now();

            // get position from camera
            robot_pose = updatePose(t, &tfBuffer);

            m_robot_pose = tf2::toMsg(robot_pose);
            float dt = (t - old_time).toSec();

            lin_speed_x = fabs(m_robot_pose.translation.x - m_old_robot_pose.translation.x) / dt;
            lin_speed_y = fabs(m_robot_pose.translation.y - m_old_robot_pose.translation.y) / dt;

            old_time = t;
            m_old_robot_pose = m_robot_pose;

            // publish the odometry over tf
            if (publish_tf)
                publish_trasform(m_robot_pose, odom_broadcaster, t);

            // publish the odometry message over ROS
            publish_odometry(m_robot_pose, odom_pub, t);
        }
        loop_rate.sleep();
        ros::spinOnce();
    }

    // close zed camera
    zed.disableTracking("./ZED_spatial_memory"); // Record an area file
    zed.close();
    return 0;
}

//------------------------------------------------------------------------------
// This function initialize variables used in odometry calculation.
//------------------------------------------------------------------------------
int
init(ros::NodeHandle node_handle) {
    // read ROS parameter from launch file
    node_handle.param<std::string>("odom_topic", odom_topic, "/visual_odom");
    node_handle.param<std::string>("base_frame_id", base_frame_id, "/base_link");
    node_handle.param<std::string>("odom_frame_id", odom_frame_id, "/odom");
    node_handle.param<std::string>("zed_frame_id", zed_frame_id, "/zed_camera");
    node_handle.param<bool>("publish_tf", publish_tf, true);
    node_handle.param<bool>("enable_spatial_memory", enable_spatial_memory, false);

    // set configuration parameters for the ZED
    InitParameters initParameters;
    initParameters.camera_resolution = RESOLUTION_HD720;
    initParameters.depth_mode = DEPTH_MODE_PERFORMANCE;
    initParameters.coordinate_units = UNIT_METER;
    initParameters.coordinate_system = COORDINATE_SYSTEM_IMAGE;
    //initParameters.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP; // ROS
    //initParameters.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP; // Zed

    // open the camera
    ERROR_CODE err = zed.open(initParameters);
    if (err != sl::SUCCESS) {
        std::cout << sl::errorCode2str(err) << std::endl;
        zed.close();
        return 1; // Quit if an error occurred
    }

    // Set positional tracking parameters
    TrackingParameters trackingParameters;
    sl::Transform initial_position;
    initial_position.setTranslation(sl::Translation(0.20,0,0.05)); // measure in m
    trackingParameters.initial_world_transform = initial_position;
    // Builds and updates a spatial representation of its surroundings when
    // discovering an environment.
    trackingParameters.enable_spatial_memory = enable_spatial_memory;

    // Start motion tracking
    zed.enableTracking(trackingParameters);

    return 0;
}

//------------------------------------------------------------------------------
// This function update the position get from zed camera and calculate linear
// velocity with old position.
//------------------------------------------------------------------------------
tf2::Transform
updatePose(ros::Time time, tf2_ros::Buffer *tfBuffer) {
    // Get the pose of the camera relative to the world frame, this is the
    // global position of the camera
    TRACKING_STATE state = zed.getPosition(camera_pose, REFERENCE_FRAME_WORLD);
    // Get the pose of the camera relative to the camera frame, this is the
    // position relative to the last position (odometry).
    //TRACKING_STATE state = zed.getPosition(zed_pose, REFERENCE_FRAME_CAMERA);

    // getPosition() outputs the position of the Camera Frame, which is
    // located on the left eye of the camera. To get the position of the
    // center of the camera, we transform the pose data into a new frame
    // located at the center of the camera.The generic formula used here is:
    // Pose(new reference frame) = M.inverse() * Pose (camera frame) * M,
    // where M is the transform between two frames.
    transformPose(camera_pose.pose_data, translation_left_to_center);

    // Get quaternion, rotation and translation
    sl::Translation translation     = camera_pose.getTranslation();
    sl::Orientation quat            = camera_pose.getOrientation();

    // Transform from base to sensor
    tf2::Transform base_to_sensor;
    // Look up the transformation from base frame to camera link
    try {
        // Save the transformation from base to frame
        geometry_msgs::TransformStamped b2s = tfBuffer->lookupTransform(base_frame_id, zed_frame_id, ros::Time(0));
        // Get the TF2 transformation
        tf2::fromMsg(b2s.transform, base_to_sensor);
    } catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(10.0, "The tf from '%s' to '%s' does not seem to be available, "
                "will assume it as identity!",
                base_frame_id.c_str(),
                zed_frame_id.c_str());
        ROS_DEBUG("Transform error: %s", ex.what());
        base_to_sensor.setIdentity();
    }

    // Transform ZED pose in TF2 Transformation
    tf2::Transform base;
    tf2::Transform camera_transform;
    geometry_msgs::Transform c2s;
    // translation
    c2s.translation.x = translation(2);
    c2s.translation.y = -translation(0);
    c2s.translation.z = -translation(1);
    // rotation
    c2s.rotation.x = quat(2);
    c2s.rotation.y = -quat(0);
    c2s.rotation.z = -quat(1);
    c2s.rotation.w = quat(3);
    tf2::fromMsg(c2s, camera_transform);
    // Transformation from camera sensor to base frame
    base = base_to_sensor * camera_transform * base_to_sensor.inverse();

    return base;
}

//------------------------------------------------------------------------------
// This function publish the transform of camera odometry in respect of
// base_frame_id
//------------------------------------------------------------------------------
void
publish_trasform(geometry_msgs::Transform robot_pose, tf2_ros::TransformBroadcaster tf_broadcaster, ros::Time time) {
    geometry_msgs::TransformStamped     odom_trans;

    {
        odom_trans.header.stamp = time;
        odom_trans.header.frame_id = odom_frame_id;
        odom_trans.child_frame_id = base_frame_id;
        odom_trans.transform = robot_pose;
    }

    //send the transform
    tf_broadcaster.sendTransform(odom_trans);
}

//------------------------------------------------------------------------------
// This function publish the odometry message over ROS as a nav_msgs message
//------------------------------------------------------------------------------
void
publish_odometry(geometry_msgs::Transform robot_pose, ros::Publisher odom_publisher, ros::Time time) {
    nav_msgs::Odometry          odom;
    geometry_msgs::Transform    base2;

    {
        odom.header.stamp = time;
        odom.header.frame_id = odom_frame_id;
        odom.child_frame_id = base_frame_id;
        // Add all value in odometry message
        odom.pose.pose.position.x = robot_pose.translation.x;
        odom.pose.pose.position.y = robot_pose.translation.y;
        odom.pose.pose.position.z = robot_pose.translation.z;
        odom.twist.twist.linear.x = lin_speed_x;
        odom.twist.twist.linear.y = lin_speed_y;
        odom.pose.pose.orientation.x = robot_pose.rotation.x;
        odom.pose.pose.orientation.y = robot_pose.rotation.y;
        odom.pose.pose.orientation.z = robot_pose.rotation.z;
        odom.pose.pose.orientation.w = robot_pose.rotation.w;
    }

    //publish the message
    odom_publisher.publish(odom);
}

//------------------------------------------------------------------------------
// Trasnform pose to create a Tracking Frame located in a separate location
// from the Camera Frame (left eye).
//------------------------------------------------------------------------------
void
transformPose(sl::Transform &pose, float tx) {
    sl::Transform transform_;
    transform_.setIdentity();
    // Move the tracking frame by tx along the X axis
    transform_.tx = tx;
    // Apply the transformation
    pose = Transform::inverse(transform_) * pose * transform_;
}
