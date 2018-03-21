/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <industrial_extrinsic_cal/target.h>
#include <industrial_extrinsic_cal/ros_camera_observer.h>  // for pattern options

namespace industrial_extrinsic_cal
{
void Target::pushTransform()
{
  transform_interface_->pushTransform(pose_);
}
void Target::pullTransform()
{
  pose_ = transform_interface_->pullTransform();
}
void Target::setTransformInterface(boost::shared_ptr<TransformInterface> transform_interface)
{
  transform_interface_ = transform_interface;
}
boost::shared_ptr<TransformInterface> Target::getTransformInterface()
{
  return (transform_interface_);
}
void Target::setTIReferenceFrame(std::string ref_frame)
{
  transform_interface_->setReferenceFrame(ref_frame);
}
void Target::generatePoints()
{
    pts_.clear();
    int rows, cols;
    double spacing;
    switch(target_type_){
    case pattern_options::Chessboard:
      rows    = checker_board_parameters_.pattern_rows;
      cols    = checker_board_parameters_.pattern_cols;
      spacing = checker_board_parameters_.square_size;
        break;
    case pattern_options::CircleGrid:
    case pattern_options::ModifiedCircleGrid:
      rows    = circle_grid_parameters_.pattern_rows;
      cols    = circle_grid_parameters_.pattern_cols;
      spacing = circle_grid_parameters_.spacing;
      break;
    default:
      break;
    }// end switch for target type

    num_points_ = rows*cols;
    for(int i=0; i<rows; i++){
      for(int j=0; j<cols; j++){
	Point3d point;
	point.x = j*spacing;
	point.y = (rows -1 -i)*spacing;
	point.z = 0.0;
	pts_.push_back(point);
      }
    }
}// end of Target::generatePoints()
  
}  // end of namespace
