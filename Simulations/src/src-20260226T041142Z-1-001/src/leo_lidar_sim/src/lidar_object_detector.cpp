#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vector>
#include <cmath>

class LidarObjectDetector : public rclcpp::Node
{
public:
  LidarObjectDetector() : Node("lidar_object_detector")
  {
    // Parameters
    this->declare_parameter("min_cluster_size", 3);
    this->declare_parameter("max_cluster_size", 100);
    this->declare_parameter("cluster_tolerance", 0.3);
    this->declare_parameter("min_object_distance", 0.2);
    this->declare_parameter("max_object_distance", 5.0);
    
    min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
    max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();
    cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
    min_distance_ = this->get_parameter("min_object_distance").as_double();
    max_distance_ = this->get_parameter("max_object_distance").as_double();

    // Subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&LidarObjectDetector::scanCallback, this, std::placeholders::_1));

    // Publishers
    objects_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/detected_objects", 10);
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/object_markers", 10);

    RCLCPP_INFO(this->get_logger(), "LiDAR Object Detector started");
  }

private:
  struct Point2D
  {
    double x;
    double y;
    
    double distance() const {
      return std::sqrt(x * x + y * y);
    }
  };

  struct Cluster
  {
    std::vector<Point2D> points;
    Point2D centroid;
    
    void calculateCentroid() {
      if (points.empty()) return;
      
      double sum_x = 0.0, sum_y = 0.0;
      for (const auto& p : points) {
        sum_x += p.x;
        sum_y += p.y;
      }
      centroid.x = sum_x / points.size();
      centroid.y = sum_y / points.size();
    }
  };

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Convert LaserScan to 2D points
    std::vector<Point2D> points;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float range = msg->ranges[i];
      
      // Filter invalid points
      if (std::isnan(range) || std::isinf(range) || 
          range < min_distance_ || range > max_distance_) {
        continue;
      }
      
      float angle = msg->angle_min + i * msg->angle_increment;
      Point2D point;
      point.x = range * std::cos(angle);
      point.y = range * std::sin(angle);
      points.push_back(point);
    }

    // Cluster points
    std::vector<Cluster> clusters = clusterPoints(points);
    
    // Publish detected objects
    publishObjects(clusters, msg->header);
    publishMarkers(clusters, msg->header);
    
    RCLCPP_INFO(this->get_logger(), "Detected %zu objects", clusters.size());
  }

  std::vector<Cluster> clusterPoints(const std::vector<Point2D>& points)
  {
    std::vector<Cluster> clusters;
    std::vector<bool> processed(points.size(), false);

    for (size_t i = 0; i < points.size(); ++i) {
      if (processed[i]) continue;

      Cluster cluster;
      std::vector<size_t> seed_queue;
      seed_queue.push_back(i);
      processed[i] = true;

      while (!seed_queue.empty()) {
        size_t current_idx = seed_queue.back();
        seed_queue.pop_back();
        cluster.points.push_back(points[current_idx]);

        // Find neighbors
        for (size_t j = 0; j < points.size(); ++j) {
          if (processed[j]) continue;

          double dx = points[current_idx].x - points[j].x;
          double dy = points[current_idx].y - points[j].y;
          double distance = std::sqrt(dx * dx + dy * dy);

          if (distance < cluster_tolerance_) {
            seed_queue.push_back(j);
            processed[j] = true;
          }
        }
      }

      // Filter clusters by size
      if (cluster.points.size() >= min_cluster_size_ && 
          cluster.points.size() <= max_cluster_size_) {
        cluster.calculateCentroid();
        clusters.push_back(cluster);
      }
    }

    return clusters;
  }

  void publishObjects(const std::vector<Cluster>& clusters, const std_msgs::msg::Header& header)
  {
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header = header;
    pose_array.header.frame_id = "lidar_link";

    for (const auto& cluster : clusters) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = cluster.centroid.x;
      pose.position.y = cluster.centroid.y;
      pose.position.z = 0.0;
      pose.orientation.w = 1.0;
      pose_array.poses.push_back(pose);
    }

    objects_pub_->publish(pose_array);
  }

  void publishMarkers(const std::vector<Cluster>& clusters, const std_msgs::msg::Header& header)
  {
    visualization_msgs::msg::MarkerArray marker_array;

    // Delete old markers
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    // Create new markers
    for (size_t i = 0; i < clusters.size(); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header = header;
      marker.header.frame_id = "lidar_link";
      marker.ns = "objects";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      
      marker.pose.position.x = clusters[i].centroid.x;
      marker.pose.position.y = clusters[i].centroid.y;
      marker.pose.position.z = 0.5;
      marker.pose.orientation.w = 1.0;
      
      marker.scale.x = 0.3;
      marker.scale.y = 0.3;
      marker.scale.z = 0.3;
      
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 0.8;
      
      marker.lifetime = rclcpp::Duration::from_seconds(0.5);
      
      marker_array.markers.push_back(marker);
    }

    markers_pub_->publish(marker_array);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr objects_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  int min_cluster_size_;
  int max_cluster_size_;
  double cluster_tolerance_;
  double min_distance_;
  double max_distance_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarObjectDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
