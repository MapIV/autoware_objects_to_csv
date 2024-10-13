#include "rclcpp/rclcpp.hpp"
#include "tier4_perception_msgs/msg/detected_objects_with_feature.hpp"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>

using namespace std;
using namespace std::chrono_literals;
using DetectedObjectsWithFeature = tier4_perception_msgs::msg::DetectedObjectsWithFeature;

class DetectedObjectsWithFeatureToCSV : public rclcpp::Node {
public:
    DetectedObjectsWithFeatureToCSV() : Node("detected_objects_with_feature_to_csv") {
      // CSV input directory path
        this->declare_parameter<std::string>("csv_directory_path", "/default/path/to/directory");
        this->declare_parameter<std::string>("input_topic", "/detected_object");
        std::string directory_path;
        this->get_parameter("csv_directory_path", csv_file_path_);
        std::string input_topic;
        this->get_parameter("input_topic", input_topic);

        RCLCPP_INFO_STREAM(this->get_logger(), "Launching DetectedObjectsWithFeatureToCSV node with directory path: " << csv_file_path_ << " and input topic: " << input_topic);

        rclcpp::QoS qos(rclcpp::KeepLast(100));
        qos.best_effort();

        subscription_ = this->create_subscription<DetectedObjectsWithFeature>(
            input_topic,
            qos,
            bind(&DetectedObjectsWithFeatureToCSV::listener_callback, this, placeholders::_1)
        );
    }

    void listener_callback(const DetectedObjectsWithFeature::SharedPtr msg) {
        double timestamp = msg->header.stamp.sec + static_cast<double>(msg->header.stamp.nanosec) / 1e9;
        ofstream csvfile(csv_file_path_ + "/" + to_string(timestamp) + ".txt", ios::out | ios::trunc);
        // std::cout << "DetectedObjectsWithFeatureToCSV::listener_callback : " << csv_file_path_ + "/" + to_string(timestamp) + ".csv" << std::endl;
        // csvfile << "timestamp,object_class,x_position,y_position,z_position,"
        //         << "x_dimension,y_dimension,z_dimension,quaternion_x,quaternion_y,quaternion_z,quaternion_w,score" << endl;

        for (auto &feature_obj : msg->feature_objects) {

            // const uint8 CAR = 1;
            // const uint8 TRUCK = 2;
            // const uint8 BUS = 3;
            // const uint8 TRAILER = 4;
            // const uint8 MOTORCYCLE = 5;
            // const uint8 BICYCLE = 6;
            // const uint8 PEDESTRIAN = 7;
            // Map to string from ObjectClassification
            std::string object_class;
            // object_class = feature_obj.object.classification.empty() ? "Unknown" : to_string(feature_obj.object.classification[0].label); \\ if you want just class ID
            if (feature_obj.object.classification[0].label == 1) {
                object_class = "car";
            }
            else if (feature_obj.object.classification[0].label == 2 || feature_obj.object.classification[0].label == 3 || feature_obj.object.classification[0].label == 4) {
                object_class = "truck";
            }
            else if (feature_obj.object.classification[0].label == 5 || feature_obj.object.classification[0].label == 6) {
                object_class = "bicycle";
            }
            else if (feature_obj.object.classification[0].label == 7) {
                object_class = "pedestrian";
            }
            else {
                object_class = "unknown";
            }

            csvfile << fixed << setprecision(9) << timestamp << " "
                    << object_class << " "
                    << feature_obj.object.kinematics.pose_with_covariance.pose.position.x << " "
                    << feature_obj.object.kinematics.pose_with_covariance.pose.position.y << " "
                    << feature_obj.object.kinematics.pose_with_covariance.pose.position.z << " "
                    << feature_obj.object.shape.dimensions.x << " "
                    << feature_obj.object.shape.dimensions.y << " "
                    << feature_obj.object.shape.dimensions.z << " "
                    << feature_obj.object.kinematics.pose_with_covariance.pose.orientation.x << " "
                    << feature_obj.object.kinematics.pose_with_covariance.pose.orientation.y << " "
                    << feature_obj.object.kinematics.pose_with_covariance.pose.orientation.z << " "
                    << feature_obj.object.kinematics.pose_with_covariance.pose.orientation.w << " " 
                    << feature_obj.object.existence_probability << endl;
        }
    }

    string to_hex_string(const std::array<uint8_t, 16>& uuid) {
        stringstream ss;
        for (auto byte : uuid) {
            ss << hex << setw(2) << setfill('0') << (int)byte;
        }
        return ss.str();
    }

    rclcpp::Subscription<DetectedObjectsWithFeature>::SharedPtr subscription_;
    std::string csv_file_path_;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = make_shared<DetectedObjectsWithFeatureToCSV>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
