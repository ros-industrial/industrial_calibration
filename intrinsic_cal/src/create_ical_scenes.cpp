#include <stdio.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
/** This function creates a set of calibration scenes for intrinsic calibration **/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "create_caljob");
  ros::NodeHandle nh("~");
  std::string caljob_filename;
  std::string camera_name;
  std::string image_topic;
  std::string target_name;
  std::string cost_type("CameraReprjErrorWithDistortionPK");
  int target_type = 2;
  int sample_rows;
  int sample_cols;
  int sample_layers;
  double min_image_percent;
  double max_image_percent;
  std::string ref_frame("world_frame");
  int image_width, image_height;
  FILE* fp;

  // essential parameters
  if (!nh.getParam("caljob_filename", caljob_filename))
  {
    ROS_ERROR("Need to define ros parameter: caljob_filename");
    exit(1);
  }
  if (!nh.getParam("target_name", target_name))
  {
    ROS_ERROR("Need to define ros parameter: target_name");
    exit(1);
  }
  if (!nh.getParam("camera_name", camera_name))
  {
    ROS_ERROR("Need to define ros parameter: camera_name");
    exit(1);
  }
  if (!nh.getParam("image_width", image_width))
  {
    ROS_ERROR("Need to define ros parameter: image_width");
    exit(1);
  }
  if (!nh.getParam("image_topic", image_topic))
  {
    ROS_ERROR("Need to define ros parameter: image_topic");
    exit(1);
  }
  if (!nh.getParam("image_height", image_height))
  {
    ROS_ERROR("Need to define ros parameter: image_height");
    exit(1);
  }
  if (!nh.getParam("sample_rows", sample_rows))
  {
    ROS_ERROR("Need to define ros parameter: sample_rows");
    exit(1);
  }
  if (!nh.getParam("sample_cols", sample_cols))
  {
    ROS_ERROR("Need to define ros parameter: sample_cols");
    exit(1);
  }
  if (!nh.getParam("sample_layers", sample_layers))
  {
    ROS_ERROR("Need to define ros parameter: sample_layers");
    exit(1);
  }
  if (!nh.getParam("min_image_percent", min_image_percent))
  {
    ROS_ERROR("Need to define ros parameter: min_image_percent");
    exit(1);
  }
  if (!nh.getParam("max_image_percent", max_image_percent))
  {
    ROS_ERROR("Need to define ros parameter: max_image_percent");
    exit(1);
  }

  // non-essential parameters
  nh.getParam("reference_frame", ref_frame);
  nh.getParam("target_type", target_type);
  nh.getParam("cost_type", cost_type);

  // check for valid parameters
  if (min_image_percent < 1.0 || min_image_percent > 99.0)
  {
    ROS_ERROR("min_image_percent not valid %f", min_image_percent);
  }
  if (max_image_percent < 2.0 || max_image_percent > 100.0 || max_image_percent < min_image_percent)
  {
    ROS_ERROR("max_image_percent not valid %f", max_image_percent);
  }
  if (sample_layers <= 1)
  {
    ROS_ERROR("sample_layers not valid");
    exit(1);
  }
  if (!(fp = fopen(caljob_filename.c_str(), "w")))
  {
    ROS_ERROR("could not open %s for writing", caljob_filename.c_str());
    exit(1);
  }
  if (target_type < 0 || target_type > 3)
  {
    ROS_ERROR("target_type not valid %d", target_type);
  }

  // write the header
  fprintf(fp, "---\n");
  fprintf(fp, "reference_frame: %s\n", ref_frame.c_str());
  fprintf(fp, "scenes:\n");
  int scene_id = 0;
  int roi_width;
  int roi_height;

  double percent_change = (max_image_percent - min_image_percent) / 100 / (sample_layers - 1.0);
  double portion;
  for (portion = min_image_percent / 100.0; portion <= max_image_percent / 100.0; portion += percent_change)
  {
    roi_width = image_width * portion;
    roi_height = image_height * portion;
    for (int i = 0; i < sample_rows; i++)
    {
      for (int j = 0; j < sample_cols; j++)
      {
        int x = i * (image_width - roi_width) / (sample_rows - 1.0);
        int y = j * (image_height - roi_height) / (sample_cols - 1.0);
        fprintf(fp, "-\n");
        fprintf(fp, "    trigger: ROS_CAMERA_OBSERVER_TRIGGER\n");
        fprintf(fp, "    trigger_parameters:\n");
        fprintf(fp, "    -\n");
        fprintf(fp, "         service_name: ObserverTrigger\n");
        fprintf(fp, "         instructions: Center target within region of interest\n");
        fprintf(fp, "         image_topic: %s\n", image_topic.c_str());
        fprintf(fp, "         target_type: %d\n", target_type);
        fprintf(fp, "         roi_min_x: %d\n", x);
        fprintf(fp, "         roi_max_x: %d\n", x + roi_width);
        fprintf(fp, "         roi_min_y: %d\n", y);
        fprintf(fp, "         roi_max_y: %d\n", y + roi_height);
        fprintf(fp, "    observations:\n");
        fprintf(fp, "    -\n");
        fprintf(fp, "        camera: %s\n", camera_name.c_str());
        fprintf(fp, "        target: %s\n", target_name.c_str());
        fprintf(fp, "        roi_x_min: %d\n", x);
        fprintf(fp, "        roi_x_max: %d\n", x + roi_width);
        fprintf(fp, "        roi_y_min: %d\n", y);
        fprintf(fp, "        roi_y_max: %d\n", y + roi_height);
        fprintf(fp, "        cost_type: %s\n", cost_type.c_str());
      }  // end each column
    }    // end each row
  }      // end each layer
  fprintf(fp, "optimization_parameters: xx\n");
  fclose(fp);
}
