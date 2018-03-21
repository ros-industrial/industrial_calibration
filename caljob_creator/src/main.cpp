#include <ros/ros.h>

#include <string>
#include <fstream>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/JointState.h>

class CalJob
{
public:
  CalJob(const std::string& outfile, const std::string& camera_name,
	 const std::string& target_name, const std::string& cost_type)
    : file_(outfile.c_str()), camera_name_(camera_name), target_name_(target_name), cost_type_(cost_type)
  {
    if (!file_) throw std::runtime_error("Could not open outfile");
    write_headers(file_);
  }

  ~CalJob()
  {
  }

  void new_joint_scene(const sensor_msgs::JointState& state)
  {
    write_joint_scene(file_, state.position);
  }

  void new_pose_scene(const tf::StampedTransform& transform)
  {
    write_pose_scene(file_, transform);
  }
  void setImageSize(int height, int width)
  {
    image_height_ = height;
    image_width_ = width;
  }

private:
  void write_headers(std::ostream& os) const
  {
    const static char* header = "---\nreference_frame: world\nscenes:\n";
    os << header;
  }

  void write_joint_scene(std::ostream& os, const std::vector<double>& joints) const
  {
    os << "-\n";
    os << "     trigger: ROS_ROBOT_JOINT_VALUES_ACTION_TRIGGER\n";
    os << "     trig_action_server: rosRobotSceneTrigger_joint_values\n";
    os << "     joint_values:\n";
    for (size_t i = 0; i < joints.size(); ++i)
    {
      os << "        - " << joints[i] << '\n';
    }
    os << "     observations:\n";
    os << "     -\n";
    os << "          camera: " << camera_name_ << "\n";
    os << "          target: modified_circle_10x10\n";
    os << "          roi_x_min: 0\n";
    os << "          roi_x_max: " << image_height_ << "\n";
    os << "          roi_y_min: 0\n";
    os << "          roi_y_max: " << image_width_ << "\n";
    os << "          cost_type: " << cost_type_   << "\n";

    return;
  }
  void write_pose_scene(std::ostream& os, tf::StampedTransform transform) const
  {
    os << "-\n";
    os << "     trigger: ROS_ROBOT_POSE_ACTION_TRIGGER\n";
    os << "     trig_action_server: rosRobotSceneTrigger_pose\n";
    os << "     pose:\n";
    tf::Vector3 v = transform.getOrigin();
    os << "        - " << v.x() << '\n';
    os << "        - " << v.y() << '\n';
    os << "        - " << v.z() << '\n';
    tf::Quaternion Quat = transform.getRotation();
    os << "        - " << Quat.x() << '\n';
    os << "        - " << Quat.y() << '\n';
    os << "        - " << Quat.z() << '\n';
    os << "        - " << Quat.w() << '\n';
    os << "     observations:\n";
    os << "     -\n";
    os << "          camera: " << camera_name_ << "\n";
    os << "          target: " << target_name_ << "\n";
    os << "          roi_x_min: 0\n";
    os << "          roi_x_max: " << image_width_ << "\n";
    os << "          roi_y_min: 0\n";
    os << "          roi_y_max: " << image_height_ << "\n";
    os << "          cost_type: " << cost_type_   << "\n";

    return;
  }
  std::ofstream file_;
  int image_height_;
  int image_width_;
  std::string camera_name_;
  std::string target_name_;
  std::string cost_type_;
};

static const std::string OPENCV_WINDOW = "Image window";

static void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Downsize the 1080p image
  cv::pyrDown(cv_ptr->image, cv_ptr->image);

  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(3);
}

// sensor_msgs::PointCloud2ConstPtr msg =
// ros::topic::waitForMessage<sensor_msgs::PointCloud2>(params_.scan_topic,ros::Duration(WAIT_MSG_DURATION));

int main(int argc, char** argv)
{
  ros::init(argc, argv, "caljob_creator");

  ros::NodeHandle nh("~");

  // Load options from param server
  std::string image_topic_name;
  int image_height;
  int image_width;
  std::string camera_name;
  std::string target_name;
  std::string cost_type;
  nh.param<std::string>("image_topic", image_topic_name, "/kinect2/rgb_rect/image");
  nh.param<int>("image_width", image_width, 640);
  nh.param<int>("image_height", image_height, 480);
  nh.param<std::string>("camera_name", camera_name, "basler1");
  nh.param<std::string>("target_name", target_name, "m10x10");
  nh.param<std::string>("cost_type", cost_type, "PosedTargetCameraReprjErrorPK");
  std::string output_file_name;
  nh.param<std::string>("output_file", output_file_name, "caljob.yaml");

  std::string joints_topic_name;
  nh.param<std::string>("joints_topic", joints_topic_name, "motoman/joint_states");

  std::string motion_type;
  nh.param<std::string>("motion_type", motion_type, "joint");

  std::string to_frame("none");
  std::string from_frame("none");
  if (motion_type == "pose")
  {
    if (!nh.getParam("to_frame", to_frame))
    {
      ROS_ERROR("must set to_frame for motion type: pose");
    }
    if (!nh.getParam("from_frame", from_frame))
    {
      ROS_ERROR("must set from_frame for motion type: pose");
    }
    ROS_INFO("to_frame: %s from_frame %s", to_frame.c_str(), from_frame.c_str());
  }
  // Handlers for images
  image_transport::ImageTransport image_trans(nh);

  // OpenCV window scopegaurd
  struct CvScopeGaurd
  {
    CvScopeGaurd()
    {
      cv::namedWindow(OPENCV_WINDOW);
    }
    ~CvScopeGaurd()
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }
  } scope_gaurd;

  // Create subscribers for images
  image_transport::Subscriber image_sub = image_trans.subscribe(image_topic_name, 1, imageCb);

  CalJob caljob(output_file_name, camera_name, target_name, cost_type);
  caljob.setImageSize(image_height, image_width);

  tf::TransformListener tf_listener;
  tf::StampedTransform tf_transform;

  // Main loop
  while (ros::ok())
  {
    bool capture_scene = false;
    bool quit = false;
    nh.getParam("capture_scene", capture_scene);
    nh.getParam("quit", quit);
    if (quit)
      break;
    else if (capture_scene)
    {
      nh.setParam("capture_scene", false);
      ROS_INFO("Attempting to capture robot state");
      // get robot joint state
      sensor_msgs::JointStateConstPtr joints =
          ros::topic::waitForMessage<sensor_msgs::JointState>(joints_topic_name, ros::Duration(1.0));

      // get robot pose state
      ros::Time now = ros::Time::now();
      while (!tf_listener.waitForTransform(from_frame, to_frame, now, ros::Duration(1.0)))
      {
        ROS_INFO("waiting for tranform from %s to  %s", from_frame.c_str(), to_frame.c_str());
      }
      tf_listener.lookupTransform(from_frame, to_frame, now, tf_transform);

      if (!joints)
      {
        ROS_ERROR("Could not capture joint states within capture timeframe");
      }
      else
      {
        if (motion_type == "joint")
        {
          caljob.new_joint_scene(*joints);
        }
        else if (motion_type == "pose")
        {
          caljob.new_pose_scene(tf_transform);
        }
        else
        {
          ROS_ERROR("motion type invalid, [joint or pose]: %s", motion_type.c_str());
        }
      }
    }

    ros::spinOnce();
  }
  ROS_INFO("Terminating caljob process");
  return 0;
}
