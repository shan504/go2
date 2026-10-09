// Test the unmodified Humble NavFn core on exported grids; no motion/action.
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>
#include "nav2_navfn_planner/navfn.hpp"
#include "nav2_costmap_2d/denoise/image_processing.hpp"

int main(int argc, char ** argv) {
  if (argc != 7 && argc != 8) {throw std::runtime_error("PGM START_X START_Y GOAL_X GOAL_Y EXPECT_PATH [DENOISE_MIN_GROUP]");}
  std::ifstream input(argv[1], std::ios::binary);
  std::string magic; int width, height, max;
  input >> magic >> width >> height >> max; input.get();
  if (magic != "P5" || max != 255) {throw std::runtime_error("Unsupported PGM");}
  std::vector<unsigned char> pixels(width * height), costs(width * height, 255);
  input.read(reinterpret_cast<char *>(pixels.data()), pixels.size());
  if (!input) {throw std::runtime_error("Incomplete PGM");}
  std::vector<std::pair<int, int>> occupied;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      auto value = pixels[(height - 1 - y) * width + x];
      costs[y * width + x] = value >= 250 ? 0 : value <= 10 ? 254 : 255;
    }
  }
  // The exact upstream Humble DenoiseLayer::removeGroups algorithm, on
  // combined pre-inflation costs. Unknown remains background and unchanged
  // for the global costmap; the configured local map has no unknown cells.
  if (argc == 8) {
    nav2_costmap_2d::Image<uint8_t> image(height, width, costs.data(), width);
    nav2_costmap_2d::MemoryBuffer buffer;
    nav2_costmap_2d::imgproc_impl::GroupsRemover remover;
    remover.removeGroups(image, buffer, nav2_costmap_2d::ConnectivityType::Way8,
      std::stoi(argv[7]), [](uint8_t value) {return value != 254 && value != 253;});
  }
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (costs[y * width + x] == 254) {occupied.emplace_back(x, y);}
      auto before = pixels[(height - 1 - y) * width + x];
      if (before == 205 && costs[y * width + x] != 255) {
        throw std::runtime_error("Denoise must not change unknown cells");
      }
    }
  }
  // Historical 0.25m regression fixture: 0.23m padded inscribed radius.
  // The production configuration now uses the user's requested 0.05m total
  // radius. This fixture tests earlier denoise/free-area defects, not it.
  for (auto [ox, oy] : occupied) {
    for (int dy = -5; dy <= 5; ++dy) {
      for (int dx = -5; dx <= 5; ++dx) {
        int x = ox + dx, y = oy + dy;
        double distance = std::hypot(dx, dy) * 0.05;
        if (x < 0 || y < 0 || x >= width || y >= height || distance > 0.25) {continue;}
        unsigned char cost = distance <= 0.23 ? 253 : 252 * std::exp(-12 * (distance - 0.23));
        auto & old = costs[y * width + x];
        if (old == 255) {if (cost >= 253) {old = cost;}}
        else {old = std::max(old, cost);}
      }
    }
  }
  int robot[2] = {std::stoi(argv[2]), std::stoi(argv[3])};
  int goal[2] = {std::stoi(argv[4]), std::stoi(argv[5])};
  if (costs[robot[1] * width + robot[0]] >= 253 || costs[goal[1] * width + goal[0]] >= 253) {
    throw std::runtime_error("Test endpoints must be free even in the broken map");
  }
  nav2_navfn_planner::NavFn planner(width, height);
  costs[robot[1] * width + robot[0]] = 0;
  planner.setCostmap(costs.data(), true, false);
  // These reversed endpoints match Humble NavfnPlanner::makePlan.
  planner.setStart(goal); planner.setGoal(robot); planner.calcNavFnAstar();
  int steps = planner.calcPath(width * height * 4);
  bool expected = std::stoi(argv[6]) != 0;
  if ((steps > 0) != expected) {throw std::runtime_error("Unexpected NavFn path result");}
  std::cout << "PASS native Humble NavFn A*: " << (expected ? "path found" : "no route reproduced")
            << "; steps=" << steps << "; inscribed radius=0.23m" << std::endl;
}
