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

#include <industrial_extrinsic_cal/calibration_job_definition.h>

#include <gtest/gtest.h>

using namespace industrial_extrinsic_cal;

TEST(IndustrialUtilsSuite, vector_compare)
{
/*
  std::vector<std::string> v1;
  std::vector<std::string> v2;
  std::vector<std::string> v3;

  // Testing empty vectors
  EXPECT_TRUE(isSimilar(v1, v2));
  EXPECT_TRUE(isSame(v1, v2));

  std::string i1 = "item_1";
  std::string i2 = "item_2";
  std::string i3 = "item_3";

  v1.push_back(i1);
  v1.push_back(i2);
  v1.push_back(i3);

  // Testing one empty, one populated
  EXPECT_FALSE(isSimilar(v1, v2));
  EXPECT_FALSE(isSame(v1, v2));

  v2.push_back(i1);
  v2.push_back(i2);

  // Testing one partially filled, one full
  EXPECT_FALSE(isSimilar(v1, v2));
  EXPECT_FALSE(isSame(v1, v2));

  v2.push_back(i3);

  // Testing same vectors
  EXPECT_TRUE(isSimilar(v1, v2));
  EXPECT_TRUE(isSame(v1, v2));

  v3.push_back(i3);
  v3.push_back(i2);
  v3.push_back(i1);

  // Testing similar but not same
  EXPECT_TRUE(isSimilar(v1, v3));
  EXPECT_FALSE(isSame(v1, v3));
*/
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
