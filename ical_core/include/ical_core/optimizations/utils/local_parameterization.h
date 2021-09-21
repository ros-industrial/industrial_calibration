#pragma once

#include <ical_core/types.h>
#include <ical_core/exceptions.h>

#include <ceres/problem.h>
#include <ceres/local_parameterization.h>
#include <Eigen/Core>

// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: sameeragarwal@google.com (Sameer Agarwal)

namespace industrial_calibration
{
/**
 * @brief Local parameterization of an Eigen quaternion
 *
 * As of version 1.12.0, Ceres does not provide implementations of auto-diff local parameterizations out of the box
 * However, Ceres uses them in unit its own unit tests. This struct is adapted directly from the following Ceres unit
 * test:
 *
 * https://ceres-solver.googlesource.com/ceres-solver/+/master/internal/ceres/local_parameterization_test.cc#345
 * blob: 636cac29a78648e823758e154681583cd81d5b25
 *
 */
struct EigenQuaternionPlus
{
  template <typename T>
  bool operator()(const T* x, const T* delta, T* x_plus_delta) const
  {
    const T norm_delta = sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
    Eigen::Quaternion<T> q_delta;
    if (norm_delta > T(0.0))
    {
      const T sin_delta_by_delta = sin(norm_delta) / norm_delta;
      q_delta.coeffs() << sin_delta_by_delta * delta[0], sin_delta_by_delta * delta[1], sin_delta_by_delta * delta[2],
          cos(norm_delta);
    }
    else
    {
      // We do not just use q_delta = [0,0,0,1] here because that is a
      // constant and when used for automatic differentiation will
      // lead to a zero derivative. Instead we take a first order
      // approximation and evaluate it at zero.
      q_delta.coeffs() << delta[0], delta[1], delta[2], T(1.0);
    }
    Eigen::Map<Eigen::Quaternion<T>> x_plus_delta_ref(x_plus_delta);
    Eigen::Map<const Eigen::Quaternion<T>> x_ref(x);
    x_plus_delta_ref = q_delta * x_ref;
    return true;
  }
};

/**
 * @brief Adds subset parameterization for all parameter blocks for a given problem
 * @param problem - Ceres optimization problem
 * @param masks - an array of masks indicating the index of the parameters that should be held constant.
 *   - The array must be the same size as the number of parameter blocks in the problem
 *   - Individual mask vectors cannot mask all parameters in a block
 * @param parameter_blocks - Ceres parameter blocks in the same order as the input masks
 * @throws OptimizationException
 */
template <std::size_t N_BLOCKS>
void addSubsetParameterization(ceres::Problem& problem, const std::array<std::vector<int>, N_BLOCKS>& masks,
                               const std::array<double*, N_BLOCKS>& parameter_blocks)
{
  // Make sure the Ceres problem has the same number of blocks as the input
  if (problem.NumParameterBlocks() != N_BLOCKS)
  {
    std::stringstream ss;
    ss << "Input parameter block size does not match Ceres problem parameter block size (" << N_BLOCKS
       << " != " << problem.NumParameterBlocks() << ")";
    throw OptimizationException(ss.str());
  }

  if (parameter_blocks.size() != masks.size())
  {
    std::stringstream ss;
    ss << "Parameter block count does not match number of masks (" << parameter_blocks.size() << " != " << masks.size();
    throw OptimizationException(ss.str());
  }

  // Set identity local parameterization on individual variables that we want to remain constant
  for (std::size_t i = 0; i < parameter_blocks.size(); ++i)
  {
    std::size_t block_size = problem.ParameterBlockSize(parameter_blocks.at(i));
    std::vector<int> mask = masks.at(i);

    if (mask.size() > 0)
    {
      // Sort the array and remove duplicate indices
      std::sort(mask.begin(), mask.end());
      auto last = std::unique(mask.begin(), mask.end());
      mask.erase(last, mask.end());

      // Check that the max index is not greater than the number of elements in the block
      auto it = std::max_element(mask.begin(), mask.end());
      if (static_cast<std::size_t>(*it) >= block_size)
      {
        std::stringstream ss;
        ss << "The largest mask index cannot be larger than or equal to the parameter block size (" << *it
           << " >= " << block_size << ")";
        throw OptimizationException(ss.str());
      }

      // Set local parameterization on the indices or set the entire block constant
      if (mask.size() >= block_size)
      {
        problem.SetParameterBlockConstant(parameter_blocks.at(i));
      }
      else
      {
        ceres::LocalParameterization* lp = new ceres::SubsetParameterization(block_size, mask);
        problem.SetParameterization(parameter_blocks[i], lp);
      }
    }
  }
}

}  // namespace industrial_calibration
