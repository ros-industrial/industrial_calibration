/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
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
 * limitations under the License.
 */

#include <industrial_extrinsic_cal/ros_transform_interface.h>
#include <iostream>
#include <fstream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "manual_calt_adjuster");
  ros::NodeHandle nh, pnh("~");
  ros::ServiceClient get_client;   /**< a client for calling the service to get the joint values associated with the
                                      transform */
  ros::ServiceClient set_client;   /**< a client for calling the service to set the joint values associated with the
                                      transform */
  ros::ServiceClient store_client; /**< a client for calling the service to store the joint values associated with the
                                      transform */
  industrial_extrinsic_cal::get_mutable_joint_states::Request get_request;     /**< request when transform is part of a
                                                                                  mutable set */
  industrial_extrinsic_cal::get_mutable_joint_states::Response get_response;   /**< response when transform is part of a
                                                                                  mutable set */
  industrial_extrinsic_cal::set_mutable_joint_states::Request set_request;     /**< request when transform is part of a
                                                                                  mutable set */
  industrial_extrinsic_cal::set_mutable_joint_states::Response set_response;   /**< response when transform is part of a
                                                                                  mutable set */
  industrial_extrinsic_cal::store_mutable_joint_states::Request store_request; /**< request to store when  part of a
                                                                                  mutable set */
  industrial_extrinsic_cal::store_mutable_joint_states::Response
      store_response; /**< response to store when  part of a mutable set */

  get_client = nh.serviceClient<industrial_extrinsic_cal::get_mutable_joint_states>("get_mutable_joint_states");
  set_client = nh.serviceClient<industrial_extrinsic_cal::set_mutable_joint_states>("set_mutable_joint_states");
  store_client = nh.serviceClient<industrial_extrinsic_cal::store_mutable_joint_states>("store_mutable_joint_states");

  std::string base_name;
  double delta_t, delta_d;
  std::vector<double> joint_values;
  if (argc == 4)
  {
    base_name = argv[1];
    sscanf(argv[2], "%lf", &delta_t);
    sscanf(argv[3], "%lf", &delta_d);
  }
  else
  {
    ROS_ERROR(" usage manual_calt_adjuster base_name delta_theta delta_distance ");
    exit(0);
  }
  get_request.joint_names.push_back(base_name + "_x_joint");
  get_request.joint_names.push_back(base_name + "_y_joint");
  get_request.joint_names.push_back(base_name + "_z_joint");
  get_request.joint_names.push_back(base_name + "_yaw_joint");
  get_request.joint_names.push_back(base_name + "_pitch_joint");
  get_request.joint_names.push_back(base_name + "_roll_joint");

  while (!get_client.call(get_request, get_response))
  {
    sleep(1);
    ROS_INFO("Waiting for mutable joint state publisher to come up");
  }
  if (get_response.joint_values.size() < 6)
  {
    ROS_ERROR("could not get the requested joint values from the mutable joint state publisher base_name: %s",
              base_name.c_str());
  }
  for (int i = 0; i < (int)get_response.joint_values.size(); i++)
  {
    joint_values.push_back(get_response.joint_values[i]);
  }

  set_request.joint_names.push_back(base_name + "_x_joint");
  set_request.joint_names.push_back(base_name + "_y_joint");
  set_request.joint_names.push_back(base_name + "_z_joint");
  set_request.joint_names.push_back(base_name + "_yaw_joint");
  set_request.joint_names.push_back(base_name + "_pitch_joint");
  set_request.joint_names.push_back(base_name + "_roll_joint");
  system("/bin/stty raw");
  ros::Rate loop_rate(10);
  char c;
  while (ros::ok() && c != 'q')
  {
    c = 'n';
    c = getc(stdin);
    set_request.joint_values.clear();
    set_request.joint_names.clear();
    switch (c)
    {
      case 'x':
        joint_values[0] -= delta_d;
        set_request.joint_values.push_back(joint_values[0]);
        set_request.joint_names.push_back(base_name + "_x_joint");
        set_client.call(set_request, set_response);
        break;
      case 'X':
        joint_values[0] += delta_d;
        set_request.joint_names.push_back(base_name + "_x_joint");
        set_request.joint_values.push_back(joint_values[0]);
        set_client.call(set_request, set_response);
        break;
      case 'y':
        joint_values[1] -= delta_d;
        set_request.joint_names.push_back(base_name + "_y_joint");
        set_request.joint_values.push_back(joint_values[1]);
        set_client.call(set_request, set_response);
        break;
      case 'Y':
        joint_values[1] += delta_d;
        set_request.joint_names.push_back(base_name + "_y_joint");
        set_request.joint_values.push_back(joint_values[1]);
        set_client.call(set_request, set_response);
        break;
      case 'z':
        joint_values[2] -= delta_d;
        set_request.joint_names.push_back(base_name + "_z_joint");
        set_request.joint_values.push_back(joint_values[2]);
        set_client.call(set_request, set_response);
        break;
      case 'Z':
        joint_values[2] += delta_d;
        set_request.joint_names.push_back(base_name + "_z_joint");
        set_request.joint_values.push_back(joint_values[2]);
        set_client.call(set_request, set_response);
        break;
      case 'w':
        joint_values[3] -= delta_t;
        set_request.joint_names.push_back(base_name + "_yaw_joint");
        set_request.joint_values.push_back(joint_values[3]);
        set_client.call(set_request, set_response);
        break;
      case 'W':
        joint_values[3] += delta_t;
        set_request.joint_names.push_back(base_name + "_yaw_joint");
        set_request.joint_values.push_back(joint_values[3]);
        set_client.call(set_request, set_response);
        break;
      case 'p':
        joint_values[4] -= delta_t;
        set_request.joint_names.push_back(base_name + "_pitch_joint");
        set_request.joint_values.push_back(joint_values[4]);
        set_client.call(set_request, set_response);
        break;
      case 'P':
        joint_values[4] += delta_t;
        set_request.joint_names.push_back(base_name + "_pitch_joint");
        set_request.joint_values.push_back(joint_values[4]);
        set_client.call(set_request, set_response);
        break;
      case 'r':
        joint_values[5] -= delta_t;
        set_request.joint_names.push_back(base_name + "_roll_joint");
        set_request.joint_values.push_back(joint_values[5]);
        set_client.call(set_request, set_response);
        break;
      case 'R':
        joint_values[5] += delta_t;
        set_request.joint_names.push_back(base_name + "_roll_joint");
        set_request.joint_values.push_back(joint_values[5]);
        set_client.call(set_request, set_response);
        break;
      case '-':
        delta_d = delta_d * .95;
        delta_t = delta_t * .95;
        break;
      case '+':
        delta_d = delta_d * 1.05;
        delta_t = delta_t * 1.05;
        break;
      case 's':
	store_client.call(store_request, store_response);
	break;
      case 'q':
	system ("/bin/stty cooked");
	exit(0);
	break;
      default:
        break;
    }  // end switch
    ros::spinOnce();
    loop_rate.sleep();
  }  // end while ok
  system("/bin/stty cooked");
}
