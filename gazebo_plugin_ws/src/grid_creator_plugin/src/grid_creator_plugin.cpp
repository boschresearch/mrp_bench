// Copyright (c) 2022 - for information on the respective copyright owner
// see the NOTICE file or the repository https://github.com/boschresearch/mrp-bench.
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

#include <iostream>
#include <fstream>
#include <math.h>
#include <limits>

// See https://github.com/ignf/gilviewer/issues/8
#define png_infopp_NULL (png_infopp) NULL
#define int_p_NULL (int*) NULL
#define png_bytep_NULL (png_bytep) NULL
#include <boost/gil.hpp>
#include <boost/gil/extension/io/png.hpp>

#include <boost/shared_ptr.hpp>
#include <sdf/sdf.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
// #include "collision_map_request.pb.h"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp> // e.g. for converting sim time to ROS time

namespace gazebo {
  // typedef const boost::shared_ptr<const collision_map_creator_msgs::msgs::CollisionMapRequest> CollisionMapRequestPtr;

  struct WorldDims {
    double minX;  // bottom left
    double minY;  // bottom left
    double maxX;  // top right
    double maxY;  // top right
  };

  class CollisionMapCreator : public WorldPlugin {
    transport::NodePtr       node;
    transport::PublisherPtr  imagePub;
    transport::SubscriberPtr commandSubscriber;
    physics::WorldPtr        world;
    // ROS pub + sub
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

  public:
    void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
      gzdbg << "SIM: Load called" << std::endl;
      world = _parent;

      std::string fileName = "map.pgm";
      // check if file already exists:
      std::ifstream infile(fileName);
      if (infile.good()) {
          gzdbg << "SIM: Map already exists. Not calculating." << std::endl;
          return;
      }
        


      // automatic map creation:
      WorldDims dims = getWorldDimensions();  // figure out max dims for ray tracing
      double resolution = 0.01;
      double height = 10.0;
      std::vector<std::vector<int>> gridVector = mapTo2DVector(dims, height, resolution);
    //   createPNGFrom2DVector(gridVector, "map_auto.png");
      createPGMFrom2DVector(gridVector, fileName);

      // convert vector to OccupancyGrid msg
      nav_msgs::msg::OccupancyGrid occupancyGrid;
      occupancyGrid.info.resolution = resolution;
      occupancyGrid.header.frame_id = "map";
      occupancyGrid.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(world->SimTime());
      occupancyGrid.info.width = gridVector[0].size();
      occupancyGrid.info.height = gridVector.size();
      occupancyGrid.info.origin.position.x = 0;
      occupancyGrid.info.origin.position.y = 0;
      occupancyGrid.info.origin.position.z = -0.01;

      std::vector<int8_t> occupancy_data;
      occupancy_data.reserve(occupancyGrid.info.width * occupancyGrid.info.height);
      for (auto row : gridVector) {
        for (auto elem : row) {
          occupancy_data.emplace_back(elem);
        }
      }
      occupancyGrid.data = occupancy_data;

      // Eigen::Quaternionf q = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX())
      // * Eigen::AngleAxisf(floorplan_img.yaw, Eigen::Vector3f::UnitZ());
      // occupancyGrid.info.origin.orientation.x = q.x();
      // occupancyGrid.info.origin.orientation.y = q.y();
      // occupancyGrid.info.origin.orientation.z = q.z();
      // occupancyGrid.info.origin.orientation.w = q.w();
      
      // Manual request
      // std::cout << "Subscribing to: "
      //           << "/collision_map/command" << std::endl;
      // commandSubscriber = node->Subscribe("/collision_map/command", &CollisionMapCreator::createMapFromRequest, this);
      // gzdbg << "SIM: Subscriber init" << std::endl;

      // ROS Publisher for OccupancyGrid
      
      // check if Node is initialized already:
      // rclcpp::shutdown();
      // int argc = 0;
      // char** argv = NULL;
      // rclcpp::init(argc, argv);

      ros_node_ = gazebo_ros::Node::Get(_sdf);
   
      // gzdbg << "SIM: rclcpp::ok(): " << rclcpp::ok() << std::endl;
      // gzdbg << "SIM: ros_node_->get_name(): " << ros_node_->get_name() << std::endl;
      // gzdbg << "SIM: ros_node_->get_namespace(): " << ros_node_->get_namespace() << std::endl;
      // auto nodes = ros_node_->get_node_names();
      // for (auto node : nodes) {
      //   gzdbg << "SIM: node: " << node << std::endl;
      // }

      rclcpp::QoS latched_qos = rclcpp::QoS(rclcpp::KeepLast(1));
      latched_qos = latched_qos.transient_local();
      latched_qos = latched_qos.reliable();
      // rclcpp::PublisherOptionsWithAllocator<Alloc> publisher_options;

      grid_pub_ = ros_node_->create_publisher<nav_msgs::msg::OccupancyGrid>("/grid", latched_qos);

      grid_pub_->publish(occupancyGrid);

      // gzdbg << "SIM: logger: " << ros_node_->get_logger().get_name() << std::endl; 
      // gzdbg << "SIM: get_topic_name: " << grid_pub_->get_topic_name() << std::endl; 

      RCLCPP_INFO(ros_node_->get_logger(), "Publishing generated grid to [%s]", grid_pub_->get_topic_name());

      // testsub
      // rclcpp::QoS sub_qos = rclcpp::QoS(rclcpp::KeepLast(1));
      // sub_qos = sub_qos.durability_volatile();
      // sub_qos = sub_qos.reliable();
      // click_sub_ = ros_node_->create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point", sub_qos,
      // std::bind(&CollisionMapCreator::testCallback, this, std::placeholders::_1));

      // gzdbg << "SIM: Spin me right round" << std::endl;

      // rclcpp::spin(ros_node_);
      // Advertise gazebo topics
      // node  = transport::NodePtr(new transport::Node());
      // node->Init(world->Name());
      // imagePub = node->Advertise<msgs::Image>("/collision_map/image");
      gzdbg << "SIM: End" << std::endl;

    }

  private:
    // void testCallback(geometry_msgs::msg::PointStamped::SharedPtr msg) {
    //   gzdbg << "SIM: Sub: " << msg->point.x << std::endl;
    // }

    // resolution = distance between two rays in m
    // height = z-coordinate
    std::vector<std::vector<int>> mapTo2DVector(WorldDims dims, double height, double resolution) {
      std::cout << "Creating collision map with corners at (" << dims.minX << ", " << dims.minY << "), (" << dims.maxX << ", " << dims.maxY << ")"
                << " with collision projected from z = " << height << "\nResolution = " << resolution << " m\n"
                << std::endl;

      int count_vertical   = abs(dims.minY - dims.maxY) / resolution;
      int count_horizontal = abs(dims.minX - dims.maxX) / resolution;

      gzdbg << "SIM: count_horizontal / num_cols: " << count_horizontal << std::endl;
      gzdbg << "SIM: count_vertical / num_rows: " << count_vertical << std::endl;

      std::vector<std::vector<int>> grid(count_vertical, std::vector<int>(count_horizontal, 0));
      // TODO: Does that size really always hold? With rounding up etc? Don't want to access index out of bounds later

      if (count_vertical == 0 || count_horizontal == 0) {
        std::cout << "Image has a zero dimensions, check coordinates" << std::endl;
        return grid;
      }

      double                   dist;
      std::string              entityName;
      ignition::math::Vector3d start, end;
      start.Z(height);
      end.Z(0.001);

      gazebo::physics::PhysicsEnginePtr engine = world->Physics();
      engine->InitForThread();
      gazebo::physics::RayShapePtr ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

      std::cout << "Rasterizing model and checking collisions" << std::endl;

      double x, y;
      x = dims.minX;
      y = dims.maxY;

      double pct_complete     = 0.0;
      int    last_printed_ten = 0;

      // ray scan row by row, column by column
      for (int i = 0; i < count_horizontal; ++i) {
        pct_complete = i * 100.0 / count_horizontal;
        if (pct_complete > last_printed_ten + 10) {
          last_printed_ten += 10;
          std::cout << "Percent complete: " << last_printed_ten << std::endl;
        }

        x += resolution;

        // ray scan row
        for (int j = 0; j < count_vertical; ++j) {
          start.X(x);
          end.X(x);
          start.Y(y);
          end.Y(y);
          ray->SetPoints(start, end);
          ray->GetIntersection(dist, entityName);
          if (!entityName.empty()) { grid[j][i] = 1; }

          y -= resolution;
        }

        y = dims.maxY;
      }

      std::cout << "Completed calculations for grid" << std::endl;
      return grid;
    }

    void createPNGFrom2DVector(std::vector<std::vector<int>> occGrid, std::string fileName) {
      std::cout << "Writing grid to " << fileName << std::endl;
      boost::gil::gray8_pixel_t fill(0);
      boost::gil::gray8_pixel_t blank(255);
      boost::gil::gray8_image_t image(occGrid[0].size(), occGrid.size());  // horizontal, vertical
      boost::gil::fill_pixels(image._view, blank);

      for (size_t i = 0; i < occGrid.size(); i++) {
        for (size_t j = 0; j < occGrid[i].size(); j++) {
          if (occGrid[i][j]) { image._view(j, i) = fill; }
        }
      }

      if (!filename.empty()) {
        boost::gil::gray8_view_t view = image._view;
        boost::gil::write_view(fileName, view, boost::gil::png_tag());
      }
      std::cout << "Writing completed." << std::endl;
    }

    void createPGMFrom2DVector(std::vector<std::vector<int>> occGrid, std::string fileName) {
      // inspired by http://aitatanit.blogspot.com/2012/03/writing-pgm-files-in-python-c-and-c.html and http://wiki.ros.org/map_server
      std::cout << "Writing grid to " << fileName << std::endl;
      uint width = occGrid[0].size();
      uint height = occGrid.size();
      unsigned char *buff = new unsigned char[width * height * sizeof(unsigned char)]; // horizontal, vertical

      for (size_t i = 0; i < height; i++) {
        for (size_t j = 0; j < width; j++) {
          if (occGrid[i][j]) { 
            // mark as occupied  
            buff[occGrid[i].size()*i + j] = 0;
          } else {
            buff[occGrid[i].size()*i + j] = 255;

          }
        }
      }

    std::ofstream fout (fileName.c_str());

    if (!fout.is_open())
    {
      std::cout << "Can't open output file"  << fileName << std::endl;
      exit(1);
    }

    // write the header
    fout << "P5\n" << width << " " << height << " 255\n";

    // write the data
    fout.write((char *)buff, width*height*sizeof(unsigned char));

    // close the stream
    fout.close();

    // free memory
    delete[] buff;
    }

    WorldDims getWorldDimensions() {
      WorldDims dims = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::min(), std::numeric_limits<double>::min()};

      std::vector<physics::ModelPtr> models = world->Models();

      // bounding boxes are world coordinates, not just relative to the models' centers
      for (auto model : models) {
        dims.minX = std::min(dims.minX, model->BoundingBox().Min().X());
        dims.minY = std::min(dims.minY, model->BoundingBox().Min().Y());
        dims.maxX = std::max(dims.maxX, model->BoundingBox().Max().X());
        dims.maxY = std::max(dims.maxY, model->BoundingBox().Max().Y());
      }
      return dims;
    }

  public:
    // void createMapFromRequest(CollisionMapRequestPtr& msg) {
    //   std::cout << "Received message" << std::endl;
    //   WorldDims dims = {msg->min().x(), msg->min().y(), msg->max().x(), msg->max().y()};

    //   std::vector<std::vector<int>> grid = mapTo2DVector(dims, msg->height(), msg->resolution());
    //   createPNGFrom2DVector(grid, msg->filename());
    // }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(CollisionMapCreator)
}  // namespace gazebo
