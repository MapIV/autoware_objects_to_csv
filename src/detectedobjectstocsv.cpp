#include "rclcpp/rclcpp.hpp"
#include "autoware_auto_perception_msgs/msg/detected_objects.hpp"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>

using namespace std;
using namespace std::chrono_literals;
using DetectedObjects = autoware_auto_perception_msgs::msg::DetectedObjects;

class DetectedObjectsToCSV : public rclcpp::Node {
public:
    DetectedObjectsToCSV() : Node("detected_objects_to_csv") {
      // CSV input directory path
        this->declare_parameter<std::string>("csv_directory_path", "/default/path/to/directory");
        this->declare_parameter<std::string>("input_topic", "/detected_object");
        std::string directory_path;
        this->get_parameter("csv_directory_path", csv_file_path_);
        std::string input_topic;
        this->get_parameter("input_topic", input_topic);

        RCLCPP_INFO_STREAM(this->get_logger(), "Launching DetectedObjectsToCSV node with directory path: " << directory_path << " and input topic: " << input_topic);

        rclcpp::QoS qos(rclcpp::KeepLast(100));
        qos.best_effort();

        subscription_ = this->create_subscription<DetectedObjects>(
            input_topic,
            qos,
            bind(&DetectedObjectsToCSV::listener_callback, this, placeholders::_1)
        );
    }

    void listener_callback(const DetectedObjects::SharedPtr msg) {
        double timestamp = msg->header.stamp.sec + static_cast<double>(msg->header.stamp.nanosec) / 1e9;
        ofstream csvfile(csv_file_path_ + to_string(timestamp) + ".csv", ios::out | ios::trunc);
        csvfile << "timestamp,object_class,x_position,y_position,z_position,"
                << "x_dimension,y_dimension,z_dimension,quaternion_x,quaternion_y,quaternion_z,quaternion_w," << endl;

        for (auto &obj : msg->objects) {
            csvfile << fixed << setprecision(9) << timestamp << ","
                    << (obj.classification.empty() ? "Unknown" : to_string(obj.classification[0].label)) << ","
                    << obj.kinematics.pose_with_covariance.pose.position.x << ","
                    << obj.kinematics.pose_with_covariance.pose.position.y << ","
                    << obj.kinematics.pose_with_covariance.pose.position.z << ","
                    << obj.shape.dimensions.x << ","
                    << obj.shape.dimensions.y << ","
                    << obj.shape.dimensions.z << ","
                    << obj.kinematics.pose_with_covariance.pose.orientation.x << ","
                    << obj.kinematics.pose_with_covariance.pose.orientation.y << ","
                    << obj.kinematics.pose_with_covariance.pose.orientation.z << ","
                    << obj.kinematics.pose_with_covariance.pose.orientation.w << "," << endl;
        }
    }

    string to_hex_string(const std::array<uint8_t, 16>& uuid) {
        stringstream ss;
        for (auto byte : uuid) {
            ss << hex << setw(2) << setfill('0') << (int)byte;
        }
        return ss.str();
    }

    rclcpp::Subscription<DetectedObjects>::SharedPtr subscription_;
    std::string csv_file_path_;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = make_shared<DetectedObjectsToCSV>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
