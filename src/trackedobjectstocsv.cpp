#include "rclcpp/rclcpp.hpp"
#include "autoware_auto_perception_msgs/msg/tracked_objects.hpp"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>

using namespace std;
using namespace std::chrono_literals;
using TrackedObjects = autoware_auto_perception_msgs::msg::TrackedObjects;

class TrackedObjectsToCSV : public rclcpp::Node {
public:
    TrackedObjectsToCSV() : Node("tracked_objects_to_csv"), sequence_number(-1) {
      // CSV input directory path
        this->declare_parameter<std::string>("csv_directory_path", "/default/path/to/directory");
        this->declare_parameter<std::string>("input_topic", "/tracked_object");
        std::string directory_path;
        this->get_parameter("csv_directory_path", directory_path);
        std::string input_topic;
        this->get_parameter("input_topic", input_topic);

        // Generate the timestamped filename
        csv_file_path_ = directory_path + "/" + generate_timestamped_filename();

        RCLCPP_INFO_STREAM(this->get_logger(), "Launching DetectedObjectsToCSV node with directory path: " << csv_file_path_ << " and input topic: " << input_topic);

        rclcpp::QoS qos(rclcpp::KeepLast(100));
        qos.best_effort();


        subscription_ = this->create_subscription<TrackedObjects>(
            input_topic,
            qos,
            bind(&TrackedObjectsToCSV::listener_callback, this, placeholders::_1)
        );

        initialize_csv();
    }

private:
    void initialize_csv() {
        ofstream csvfile(csv_file_path_, ios::out | ios::trunc);
        csvfile << "timestamp,sequence_number,object_id,object_class,x_position,y_position,z_position,"
                << "x_dimension,y_dimension,z_dimension,quaternion_x,quaternion_y,quaternion_z,quaternion_w,"
                << "velocity_x,velocity_y,velocity_z" << endl;
    }

    void listener_callback(const TrackedObjects::SharedPtr msg) {
        ofstream csvfile(csv_file_path_, ios::out | ios::app);
        sequence_number++;

        for (auto &obj : msg->objects) {
            double timestamp = msg->header.stamp.sec + static_cast<double>(msg->header.stamp.nanosec) / 1e9;

            csvfile << fixed << setprecision(9) << timestamp << ","
                    << sequence_number << ","
                    << to_hex_string(obj.object_id.uuid) << ","
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
                    << obj.kinematics.pose_with_covariance.pose.orientation.w << ","
                    << obj.kinematics.twist_with_covariance.twist.linear.x << ","
                    << obj.kinematics.twist_with_covariance.twist.linear.y << ","
                    << obj.kinematics.twist_with_covariance.twist.linear.z << endl;
        }
    }

    string to_hex_string(const std::array<uint8_t, 16>& uuid) {
        stringstream ss;
        for (auto byte : uuid) {
            ss << hex << setw(2) << setfill('0') << (int)byte;
        }
        return ss.str();
    }

    std::string generate_timestamped_filename() {
        // Get the current time
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);

        // Format the time into a string
        std::stringstream ss;
        ss << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S") << ".csv";

        return ss.str();
    }

    rclcpp::Subscription<TrackedObjects>::SharedPtr subscription_;
    int sequence_number;
    std::string csv_file_path_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = make_shared<TrackedObjectsToCSV>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
