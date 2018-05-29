#ifndef CONICAL_POSE_GENERATOR_H
#define CONICAL_POSE_GENERATOR_H

#include <Eigen/Dense>
#include <eigen_stl_containers/eigen_stl_vector_container.h>

EigenSTL::vector_Affine3d getConicalPoses(const int n,
                                          //height of cone
                             const double standoff,
                             //top of cone radius
                             const double radius);

#endif // CONICAL_POSE_GENERATOR_H
