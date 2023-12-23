#include "rclcpp/rclcpp.hpp"

#include "consai_visualizer_msgs/msg/objects.hpp"
#include "consai_visualizer_msgs/msg/shape_line.hpp"

class Pose2D {
public:
  double x;
  double y;
  double theta;

  Pose2D();
  Pose2D(double x, double y, double theta);
};

class TimeStamp {
public:
    /* 原点となる時刻　UnixTime形式 */
    uint64_t start_time_ms;
    /* start_time_ms からの相対的な時刻 */
    uint64_t timestamp_ms;

    TimeStamp();
    TimeStamp(uint64_t start_time_ms, uint64_t timestamp_ms);
};

class Pose2DStamped {
public:
  Pose2D pose;
  TimeStamp timestamp;

  Pose2DStamped();
  Pose2DStamped(Pose2D pose, TimeStamp timestamp);
};

class Trajectory {
public:
  Trajectory();
  Trajectory(std::vector<Pose2D> poses, uint64_t dt_ms);
    
  std::vector<Pose2D> poses;
  uint16_t dt_ms;
};





/**
 * Visualize 関連
*/
class TrajectoryVisualizer {
    public:
        /**
         * @brief TrajectoryからObjetcsを生成するクラスメソッド
         * @param trajectory
         * @return Objects
         */ 
        static consai_visualizer_msgs::msg::Objects createObjectsFromTrajectory(const Trajectory& trajectory);
};
