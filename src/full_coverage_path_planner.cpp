//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
#include <list>
#include <vector>

#include "full_coverage_path_planner/full_coverage_path_planner.h"

/*  *** Note the coordinate system ***
 *  grid[][] is a 2D-vector:
 *  where ix is column-index and x-coordinate in map,
 *  iy is row-index and y-coordinate in map.
 *
 *            Cols  [ix]
 *        _______________________
 *       |__|__|__|__|__|__|__|__|
 *       |__|__|__|__|__|__|__|__|
 * Rows  |__|__|__|__|__|__|__|__|
 * [iy]  |__|__|__|__|__|__|__|__|
 *       |__|__|__|__|__|__|__|__|
 *y-axis |__|__|__|__|__|__|__|__|
 *   ^   |__|__|__|__|__|__|__|__|
 *   ^   |__|__|__|__|__|__|__|__|
 *   |   |__|__|__|__|__|__|__|__|
 *   |   |__|__|__|__|__|__|__|__|
 *
 *   O   --->> x-axis
 */

// #define DEBUG_PLOT

// Default Constructor
namespace full_coverage_path_planner
{
FullCoveragePathPlanner::FullCoveragePathPlanner() : initialized_(false)
{
}

void FullCoveragePathPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return;
  }

  // create a message for the plan
  nav_msgs::Path gui_path;
  gui_path.poses.resize(path.size());

  if (!path.empty())
  {
    gui_path.header.frame_id = path[0].header.frame_id;
    gui_path.header.stamp = path[0].header.stamp;
  }

  // Extract the plan in world co-ordinates, we assume the path is all in the same frame
  for (unsigned int i = 0; i < path.size(); i++)
  {
    gui_path.poses[i] = path[i];
  }

  plan_pub_.publish(gui_path);
}

void FullCoveragePathPlanner::parsePointlist2Plan(const geometry_msgs::PoseStamped& start,
    std::list<Point_t> const& goalpoints,
    std::vector<geometry_msgs::PoseStamped>& plan)
{
  geometry_msgs::PoseStamped new_goal;
  std::list<Point_t>::const_iterator it, it_next, it_prev;
  int dx_now, dy_now, dx_next, dy_next, move_dir_now = 0, move_dir_prev = 0, move_dir_next = 0;
  float orientation = eDirNone;
  ROS_INFO("Received goalpoints with length: %lu", goalpoints.size());
  if (goalpoints.size() > 1)
  {
    for (it = goalpoints.begin(); it != goalpoints.end(); ++it)
    {
      it_next = it;
      it_next++;
      it_prev = it;
      it_prev--;

      // Check for the direction of movement
      if (it == goalpoints.begin())
      {
        dx_now = it_next->x - it->x;
        dy_now = it_next->y - it->y;
      }
      else
      {
        dx_now = it->x - it_prev->x;
        dy_now = it->y - it_prev->y;
        dx_next = it_next->x - it->x;
        dy_next = it_next->y - it->y;
      }

      // Calculate direction enum: dx + dy*2 will give a unique number for each of the four possible directions because
      // of their signs:
      //  1 +  0*2 =  1
      //  0 +  1*2 =  2
      // -1 +  0*2 = -1
      //  0 + -1*2 = -2
      move_dir_now = dx_now + dy_now * 2;
      move_dir_next = dx_next + dy_next * 2;
      move_dir_prev = move_dir_now;

      // Add to vector
      new_goal.header.frame_id = map_frame_;
      new_goal.pose.position.x = (it->x) * tile_size_ + grid_origin_.x + tile_size_ * 0.5;
      new_goal.pose.position.y = (it->y) * tile_size_ + grid_origin_.y + tile_size_ * 0.5;

      // Calculate desired orientation to be in line with movement direction
      switch (move_dir_now)
      {
        case eDirNone:
          // Keep orientation
          break;
        case eDirRight:
          orientation = 0;
          break;
        case eDirUp:
          orientation = M_PI / 2;
          break;
        case eDirLeft:
          orientation = M_PI;
          break;
        case eDirDown:
          orientation = M_PI * 1.5;
          break;
      }
      new_goal.pose.orientation = tf::createQuaternionMsgFromYaw(orientation);
      if (it != goalpoints.begin())
      {
        previous_goal_.pose.orientation = new_goal.pose.orientation;
        // republish previous goal but with new orientation to indicate change of direction
        // useful when the plan is strictly followed with base_link
        plan.push_back(previous_goal_);
      }
      ROS_DEBUG("Voila new point: x=%f, y=%f, o=%f,%f,%f,%f", new_goal.pose.position.x, new_goal.pose.position.y,
                new_goal.pose.orientation.x, new_goal.pose.orientation.y, new_goal.pose.orientation.z,
                new_goal.pose.orientation.w);
      plan.push_back(new_goal);
      previous_goal_ = new_goal;
    }
  }
  else
  {
    new_goal.header.frame_id = map_frame_;
    new_goal.pose.position.x = (goalpoints.begin()->x) * tile_size_ + grid_origin_.x + tile_size_ * 0.5;
    new_goal.pose.position.y = (goalpoints.begin()->y) * tile_size_ + grid_origin_.y + tile_size_ * 0.5;
    new_goal.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    plan.push_back(new_goal);
  }
  /* Add poses from current position to start of plan */

  // Compute angle between current pose and first plan point
  double dy = plan.begin()->pose.position.y - start.pose.position.y;
  double dx = plan.begin()->pose.position.x - start.pose.position.x;
  // Arbitrary choice of 100.0*FLT_EPSILON to determine minimum angle precision of 1%
  if (!(fabs(dy) < 100.0 * FLT_EPSILON && fabs(dx) < 100.0 * FLT_EPSILON))
  {
    // Add extra translation waypoint
    double yaw = std::atan2(dy, dx);
    geometry_msgs::Quaternion quat_temp = tf::createQuaternionMsgFromYaw(yaw);
    geometry_msgs::PoseStamped extra_pose;
    extra_pose = *plan.begin();
    extra_pose.pose.orientation = quat_temp;
    plan.insert(plan.begin(), extra_pose);
    extra_pose = start;
    extra_pose.pose.orientation = quat_temp;
    plan.insert(plan.begin(), extra_pose);
  }

  // Insert current pose
  plan.insert(plan.begin(), start);

  ROS_INFO("Plan ready containing %lu goals!", plan.size());
}

bool FullCoveragePathPlanner::parseGrid(nav_msgs::OccupancyGrid const& cpp_grid_,
                                        std::vector<std::vector<bool> >& grid,
                                        float robotRadius,
                                        float toolRadius,
                                        geometry_msgs::PoseStamped const& realStart,
                                        Point_t& scaledStart)
{
  int ix, iy, nodeRow, nodeColl;
  uint32_t nodeSize = dmax(floor(toolRadius / cpp_grid_.info.resolution), 1);  // Size of node in pixels/units
  uint32_t robotNodeSize = dmax(floor(robotRadius / cpp_grid_.info.resolution), 1);  // RobotRadius in pixels/units
  uint32_t nRows = cpp_grid_.info.height, nCols = cpp_grid_.info.width;
  ROS_INFO("nRows: %u nCols: %u nodeSize: %d", nRows, nCols, nodeSize);

  if (nRows == 0 || nCols == 0)
  {
    return false;
  }

  // Save map origin and scaling
  tile_size_ = nodeSize * cpp_grid_.info.resolution;  // Size of a tile in meters
  grid_origin_.x = cpp_grid_.info.origin.position.x;  // x-origin in meters
  grid_origin_.y = cpp_grid_.info.origin.position.y;  // y-origin in meters

  // Set map frame id
  map_frame_ = cpp_grid_.header.frame_id;

  // Scale starting point
  scaledStart.x = static_cast<unsigned int>(clamp((realStart.pose.position.x - grid_origin_.x) / tile_size_, 0.0,
                             floor(cpp_grid_.info.width / tile_size_)));
  scaledStart.y = static_cast<unsigned int>(clamp((realStart.pose.position.y - grid_origin_.y) / tile_size_, 0.0,
                             floor(cpp_grid_.info.height / tile_size_)));

  // Scale grid
  for (iy = 0; iy < nRows; iy = iy + nodeSize)
  {
    std::vector<bool> gridRow;
    for (ix = 0; ix < nCols; ix = ix + nodeSize)
    {
      bool nodeOccupied = false;
      for (nodeRow = 0; (nodeRow < robotNodeSize) && ((iy + nodeRow) < nRows) && (nodeOccupied == false); ++nodeRow)
      {
        for (nodeColl = 0; (nodeColl < robotNodeSize) && ((ix + nodeColl) < nCols); ++nodeColl)
        {
          int index_grid = dmax((iy + nodeRow - ceil(static_cast<float>(robotNodeSize - nodeSize) / 2.0))
                            * nCols + (ix + nodeColl - ceil(static_cast<float>(robotNodeSize - nodeSize) / 2.0)), 0);
          if (cpp_grid_.data[index_grid] > 65)
          {
            nodeOccupied = true;
            break;
          }
        }
      }
      gridRow.push_back(nodeOccupied);
    }
    grid.push_back(gridRow);
  }
  return true;
}

void FullCoveragePathPlanner::upsamplePlan(std::vector<geometry_msgs::PoseStamped>& path, double seperation)
{
  if(path.size() < 2) return;

  std::vector<geometry_msgs::PoseStamped> path_ = path;
  path.clear();

  int count = 0;

  for(int i = 0; i < path_.size()-1; i++)
  {
    geometry_msgs::PoseStamped pose1 = path_[i];
    geometry_msgs::PoseStamped pose2 = path_[i+1];

    // Add pose1 to new path
    pose1.header.seq = count++;
    pose1.header.stamp = ros::Time::now();
    path.push_back(pose1);

    // If poses have same orientation, do not upsample
    if(tf::getYaw(pose1.pose.orientation) != tf::getYaw(pose2.pose.orientation)) continue;

    // Find eucledian distance
    double dx, dy, distance;
    dx = pose1.pose.position.x - pose2.pose.position.x;
    dy = pose1.pose.position.y - pose2.pose.position.y;
    distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

    // Check if distance is greater than seperation
    if(distance <= seperation) continue;

    // Find intermediate pose count
    int pose_count = distance / seperation;

    // Upsample poses
    for(int j = 1; j <= pose_count; j++)
    {
      geometry_msgs::PoseStamped pose = pose1;

      pose.pose.position.x -= (dx / pose_count * j);
      pose.pose.position.y -= (dy / pose_count * j);

      pose.header.seq = count++;
      pose.header.stamp = ros::Time::now();

      path.push_back(pose);
    }
  }

  geometry_msgs::PoseStamped last_pose = path[path.size()-1];
  last_pose.header.seq = count++;
  last_pose.header.stamp = ros::Time::now();

  path.push_back(path[path.size()-1]);
}
}  // namespace full_coverage_path_planner
