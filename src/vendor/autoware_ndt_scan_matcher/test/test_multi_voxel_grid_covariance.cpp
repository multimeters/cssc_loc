// Copyright 2026 Libpet
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/ndt_scan_matcher/ndt_omp/multi_voxel_grid_covariance_omp.h"

#include <gtest/gtest.h>
#include <pcl/point_types.h>

#include <utility>

namespace
{

using VoxelGrid = pclomp::MultiVoxelGridCovariance<pcl::PointXYZ>;

TEST(MultiVoxelGridCovariance, default_grid_has_an_empty_voxel_cloud)
{
  const VoxelGrid grid;

  EXPECT_TRUE(grid.getVoxelPCD().empty());
}

TEST(MultiVoxelGridCovariance, moved_from_grid_has_an_empty_voxel_cloud)
{
  VoxelGrid source;
  const VoxelGrid destination{std::move(source)};

  EXPECT_TRUE(source.getVoxelPCD().empty());
  EXPECT_TRUE(destination.getVoxelPCD().empty());
}

}  // namespace
