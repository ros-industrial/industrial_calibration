#include <industrial_extrinsic_cal/conical_pose_generator.h>

EigenSTL::vector_Affine3d getConicalPoses(const int n,
                             //height of cone
                             const double standoff,
                             //top of cone radius
                             const double radius)
{
  EigenSTL::vector_Affine3d frames;

  double dt = 2.0f * M_PI / double(n);
  double dp = std::tan(radius / standoff);
  

  for(int i = 0; i < n; ++i)
  {
    Eigen::Affine3d frame (Eigen::Affine3d::Identity());
    //Move to the correct height
    frame.translate(Eigen::Vector3d {0.0f, 0.0f, standoff});
    //Rotate about z to each angle by incrimenting 
    frame.rotate(Eigen::AngleAxisd {double(i) * dt, Eigen::Vector3d::UnitZ()});
    //Move to the correct radius
    frame.translate(Eigen::Vector3d {radius, 0.0f, 0.0f});
    //rotate about Y to make z point at big dot
    frame.rotate(Eigen::AngleAxisd {dp, Eigen::Vector3d::UnitY()});
    //rotate about x to point z at big dot
    frame.rotate(Eigen::AngleAxisd {M_PI, Eigen::Vector3d::UnitX()});

    // once the camera is looking at the point with axis z
    // rotate camera about z axis to get 3 more positions
    for(int i = 0; i <= 2; ++i)
    {
       if (i = 0)
       {
         frames.push_back(std::move(frame));
       }
       if (i = 1)
       {
        frame.rotate(Eigen::AngleAxisd {M_PI/4, Eigen::Vector3d::UnitZ()});
	frames.push_back(std::move(frame));
       }
       if (i = 2)
       {
	 frame.rotate(Eigen::AngleAxisd {-M_PI/2, Eigen::Vector3d::UnitZ()});
	 frames.push_back(std::move(frame));
       }
	
    }
  }

  // Add a frame looking directly down
  Eigen::Affine3d frame (Eigen::Affine3d::Identity());
  frame.translate(Eigen::Vector3d {0.0f, 0.0f, standoff});
  frame.rotate(Eigen::AngleAxisd {M_PI, Eigen::Vector3d::UnitX()});
  frames.push_back(std::move(frame));

  return frames;
}


