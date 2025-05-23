#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <sound_play/SoundRequest.h>

class DistanceMonitor {
public:
    DistanceMonitor() : nh_("~") {
        // 读取参数
        nh_.param<float>("min_safe_distance", min_safe_distance_, 2.0);
        nh_.param<float>("warning_distance", warning_distance_, 3.0);
        nh_.param<float>("critical_distance", critical_distance_, 1.0);
        nh_.param<bool>("enable_sound_alerts", enable_sound_alerts_, true);
        nh_.param<bool>("enable_text_alerts", enable_text_alerts_, true);
        
        // 初始化发布器和订阅器
        distance_sub_ = nh_.subscribe("min_distance", 1, &DistanceMonitor::distanceCallback, this);
        alert_pub_ = nh_.advertise<std_msgs::String>("safety_alerts", 1);
        
        if (enable_sound_alerts_) {
            sound_pub_ = nh_.advertise<sound_play::SoundRequest>("/robotsound", 1);
        }
        
        last_alert_time_ = ros::Time(0);
        current_distance_ = std::numeric_limits<float>::max();
        
        ROS_INFO("Distance Monitor initialized:");
        ROS_INFO("  - Min safe distance: %.2f m", min_safe_distance_);
        ROS_INFO("  - Warning distance: %.2f m", warning_distance_);
        ROS_INFO("  - Critical distance: %.2f m", critical_distance_);
        ROS_INFO("  - Sound alerts: %s", enable_sound_alerts_ ? "enabled" : "disabled");
        ROS_INFO("  - Text alerts: %s", enable_text_alerts_ ? "enabled" : "disabled");
    }

private:
    void distanceCallback(const std_msgs::Float32::ConstPtr& msg) {
        current_distance_ = msg->data;
        
        // 检查安全状态
        SafetyLevel level = evaluateSafetyLevel(current_distance_);
        
        // 发布报警信息
        publishAlert(level);
        
        // 限制报警频率（每秒最多一次）
        ros::Time current_time = ros::Time::now();
        if ((current_time - last_alert_time_).toSec() >= 1.0) {
            if (level != SAFE) {
                if (enable_sound_alerts_) {
                    publishSoundAlert(level);
                }
                last_alert_time_ = current_time;
            }
        }
        
        // 记录状态变化
        if (level != last_safety_level_) {
            logSafetyStatusChange(level, current_distance_);
            last_safety_level_ = level;
        }
    }
    
    enum SafetyLevel {
        SAFE,
        WARNING,
        DANGER,
        CRITICAL
    };
    
    SafetyLevel evaluateSafetyLevel(float distance) {
        if (distance <= critical_distance_) {
            return CRITICAL;
        } else if (distance <= min_safe_distance_) {
            return DANGER;
        } else if (distance <= warning_distance_) {
            return WARNING;
        } else {
            return SAFE;
        }
    }
    
    void publishAlert(SafetyLevel level) {
        if (!enable_text_alerts_) return;
        
        std_msgs::String alert_msg;
        
        switch (level) {
            case CRITICAL:
                alert_msg.data = "🚨 CRITICAL: Obstacle too close to powerline! Distance: " + 
                               std::to_string(current_distance_).substr(0, 4) + "m";
                break;
            case DANGER:
                alert_msg.data = "⚠️ DANGER: Unsafe distance to powerline! Distance: " + 
                               std::to_string(current_distance_).substr(0, 4) + "m";
                break;
            case WARNING:
                alert_msg.data = "⚡ WARNING: Close to powerline. Distance: " + 
                               std::to_string(current_distance_).substr(0, 4) + "m";
                break;
            case SAFE:
                alert_msg.data = "✅ SAFE: Distance to powerline: " + 
                               std::to_string(current_distance_).substr(0, 4) + "m";
                break;
        }
        
        alert_pub_.publish(alert_msg);
    }
    
    void publishSoundAlert(SafetyLevel level) {
        sound_play::SoundRequest sound_req;
        sound_req.sound = sound_play::SoundRequest::PLAY_STOP;
        sound_req.command = sound_play::SoundRequest::PLAY_ONCE;
        
        switch (level) {
            case CRITICAL:
                sound_req.sound = sound_play::SoundRequest::NEEDS_UNPLUGGING;
                sound_req.volume = 1.0;
                break;
            case DANGER:
                sound_req.sound = sound_play::SoundRequest::NEEDS_PLUGGING;
                sound_req.volume = 0.8;
                break;
            case WARNING:
                sound_req.sound = sound_play::SoundRequest::BACKINGUP;
                sound_req.volume = 0.6;
                break;
            default:
                return;  // 安全状态不发声
        }
        
        sound_pub_.publish(sound_req);
    }
    
    void logSafetyStatusChange(SafetyLevel level, float distance) {
        std::string level_str;
        std::string color_code;
        
        switch (level) {
            case CRITICAL:
                level_str = "CRITICAL";
                color_code = "\033[1;31m";  // 红色加粗
                break;
            case DANGER:
                level_str = "DANGER";
                color_code = "\033[1;33m";  // 黄色加粗
                break;
            case WARNING:
                level_str = "WARNING";
                color_code = "\033[1;36m";  // 青色加粗
                break;
            case SAFE:
                level_str = "SAFE";
                color_code = "\033[1;32m";  // 绿色加粗
                break;
        }
        
        ROS_INFO("%s[SAFETY STATUS] %s: Distance = %.2f m\033[0m", 
                 color_code.c_str(), level_str.c_str(), distance);
        
        // 如果是危险状态，额外输出详细信息
        if (level >= DANGER) {
            ROS_WARN("Action required: Obstacle at %.2f m from powerline (threshold: %.2f m)", 
                     distance, min_safe_distance_);
        }
    }
    
    // ROS相关
    ros::NodeHandle nh_;
    ros::Subscriber distance_sub_;
    ros::Publisher alert_pub_;
    ros::Publisher sound_pub_;
    
    // 参数
    float min_safe_distance_;
    float warning_distance_;
    float critical_distance_;
    bool enable_sound_alerts_;
    bool enable_text_alerts_;
    
    // 状态变量
    float current_distance_;
    SafetyLevel last_safety_level_;
    ros::Time last_alert_time_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_monitor_node");
    
    try {
        DistanceMonitor monitor;
        ROS_INFO("Distance Monitor started. Monitoring powerline safety...");
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in distance monitor: %s", e.what());
        return -1;
    }
    
    return 0;
}
