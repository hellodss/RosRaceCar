#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>
#include <vector>
#include <algorithm>
#include <cmath>
class ObstacleAvoidancePID {
private:
    // ROS通信句柄
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber laser_sub_;
    // PID参数
    struct PIDParams {
        double Kp = 15.0;
        double Ki = 0.1;
        double Kd = 3.0;
        double integral = 0.0;
        double last_error = 0.0;
        double integral_max = 8.0;
    } pid_params_;
    // 配置参数
    struct Config {
        double min_range = 0.5;
        double max_range = 2.5;
        double range_threshold = 2.0;
        int frequency = 1440;
        int speed = 1825;
    } config_;
    // 极坐标点
    struct PolarPoint {
        double range = 0.0;
        double theta = 0.0;
    };
    // 直角坐标点
    struct RectangularPoint {
        double x = 0.0;
        double y = 0.0;
    };
    // PID控制计算
    double calculatePID(double error) {
        pid_params_.integral += error;
        
        // 积分限幅
        pid_params_.integral = std::min(pid_params_.integral, pid_params_.integral_max);
        
        double pid_output = pid_params_.Kp * error + 
                            pid_params_.Ki * pid_params_.integral + 
                            pid_params_.Kd * (error - pid_params_.last_error);
        
        pid_params_.last_error = error;
        return pid_output;
    }
    // 处理激光数据
    void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& laser_scan) {
        std::vector<PolarPoint> polar_points;
        std::vector<RectangularPoint> rect_points;
        
        // 检测潜在障碍物
        for (size_t i = 1; i < config_.frequency; ++i) {
            // 安全检查
            if (i >= laser_scan->ranges.size()) break;
            
            // 检测突变
            if (std::abs(laser_scan->ranges[i-1] - laser_scan->ranges[i]) >= config_.range_threshold) {
                if (laser_scan->ranges[i] > config_.min_range && 
                    laser_scan->ranges[i] < config_.max_range) {
                    
                    PolarPoint point;
                    point.range = laser_scan->ranges[i];
                    point.theta = i * laser_scan->angle_increment + laser_scan->angle_min;
                    polar_points.push_back(point);
                }
            }
        }
        // 转换坐标
        for (auto& point : polar_points) {
            RectangularPoint rect_point;
            rect_point.x = point.range * std::sin(point.theta);
            rect_point.y = point.range * std::cos(point.theta);
            rect_points.push_back(rect_point);
        }
        // 计算误差
        double positive_sum = 0.0, negative_sum = 0.0;
        int positive_count = 0, negative_count = 0;
        for (const auto& point : rect_points) {
            if (point.x > 0) {
                positive_sum += point.x;
                positive_count++;
            } else {
                negative_sum += point.x;
                negative_count++;
            }
        }
        // 计算误差并发布速度
        double error = (positive_sum + negative_sum);
        double pid_output = calculatePID(error);
        publishVelocity(pid_output);
    }
    // 发布速度指令
    void publishVelocity(double pid_output) {
        geometry_msgs::Twist twist;
        
        // 限制角速度范围
twist.angular.z = std::max(0.0, std::min(180.0, 90 + pid_output)) * 0.001;
        twist.linear.x = config_.speed  * 0.001;
        // 重置其他轴
        twist.angular.x = twist.angular.y =twist.linear.y = twist.linear.z = 0.0;
        cmd_vel_pub_.publish(twist);
    }
public:
    ObstacleAvoidancePID() {
        // 初始化发布者和订阅者
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        laser_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(
            "/scan", 10, &ObstacleAvoidancePID::processLaserScan, this
        );
    }
};
int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "obstacle_avoidance_node");
    
    ObstacleAvoidancePID obstacle_avoider;
    
    ros::spin();
    return 0;
}