#include "gpr_perception/occupancy_grid_mapper.hpp"
#include "gtest/gtest.h"

TEST(OccupancyGridMapper, ExportsGridMap)
{
  gpr_perception::OccupancyGridConfig config;
  config.scan_area = {-1.0, 1.0, -1.0, 1.0};
  config.resolution = 0.1;
  gpr_perception::OccupancyGridMapper mapper(config);
  mapper.integrate_ray(0.0, 0.0, 0.5, 0.0, true);
  const auto grid = mapper.to_grid_map();
  EXPECT_GT(grid.width, 0);
  EXPECT_EQ(grid.data.size(), static_cast<std::size_t>(grid.width * grid.height));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
