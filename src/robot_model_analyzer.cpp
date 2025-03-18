#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <std_msgs/msg/string.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/collision/broadphase.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <urdf_parser/urdf_parser.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <memory>

class RobotModelAnalyzer : public rclcpp::Node {
public:
    RobotModelAnalyzer() : Node("robot_model_analyzer") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_description", 10,
            std::bind(&RobotModelAnalyzer::urdf_callback, this, std::placeholders::_1));
        
        this->declare_parameter<std::string>("ee_frame","gripper_link");
        this->get_parameter("ee_frame", ee_frame_);

        package_name_ = "robot_model_analyzer";
        RCLCPP_INFO(this->get_logger(), "End-effector frame: %s", ee_frame_.c_str());

        try {
            package_path_ = ament_index_cpp::get_package_share_directory(package_name_);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to find package directory: %s", e.what());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Fetching robot description from parameter server...");

        auto client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");

        while (!client->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for robot_state_publisher parameter service...");
        }

        if (client->has_parameter("robot_description")) {
            std::string urdf = client->get_parameter<std::string>("robot_description");
            RCLCPP_INFO(this->get_logger(), "Successfully retrieved robot_description from parameter server.");

            auto robot = urdf::parseURDF(urdf);
            if (!robot) {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF.");
                return;
            }

            save_urdf_to_file(urdf);
            run_pinocchio();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'robot_description' not found in robot_state_publisher.");
        }
    }

private:
    void urdf_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received robot description via topic.");

        try {
            auto robot = urdf::parseURDF(msg->data);
            if (!robot) {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF.");
                return;
            }

            save_urdf_to_file(msg->data);
            run_pinocchio();
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception while parsing URDF: %s", e.what());
        }
    }

    void save_urdf_to_file(const std::string &urdf_data) {
        if (package_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot save URDF: package path not set.");
            return;
        }

        std::string description_dir = package_path_ + "/description";
        struct stat info;
        if (stat(description_dir.c_str(), &info) != 0) {
            if (mkdir(description_dir.c_str(), 0775) != 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create directory: %s", description_dir.c_str());
                return;
            }
        }

        urdf_file_path_ = description_dir + "/temp.urdf";
        std::ofstream urdf_file(urdf_file_path_);
        if (!urdf_file) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open URDF file for writing: %s", urdf_file_path_.c_str());
            return;
        }

        urdf_file << urdf_data;
        urdf_file.close();

        RCLCPP_INFO(this->get_logger(), "URDF saved successfully at: %s", urdf_file_path_.c_str());
    }

    void run_pinocchio() {
        if (package_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Package path not set. Cannot run Pinocchio.");
            return;
        }

        std::filesystem::path urdf_path = std::filesystem::path(package_path_) / "description" / "temp.urdf";

        pinocchio::Model model;
        try {
            pinocchio::urdf::buildModel(urdf_path.string(), model);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to build Pinocchio model: %s", e.what());
            return;
        }

        pinocchio::GeometryModel visual_model, collision_model;
        pinocchio::urdf::buildGeom(model, urdf_path.string(), pinocchio::VISUAL, visual_model);
        pinocchio::urdf::buildGeom(model, urdf_path.string(), pinocchio::COLLISION, collision_model);
        collision_model.addAllCollisionPairs();

        pinocchio::Data data(model);
        pinocchio::GeometryData collision_data(collision_model);

        Eigen::VectorXd q = pinocchio::randomConfiguration(model);

        if (!model.existFrame(ee_frame_)) {
            RCLCPP_ERROR(this->get_logger(), "Frame '%s' not found in model.", ee_frame_.c_str());
            return;
        }

        const auto ee_frame_id = model.getFrameId(ee_frame_);
        pinocchio::framesForwardKinematics(model, data, q);

        std::cout << "End-effector transform:\n" << data.oMf[ee_frame_id] << "\n";

        Eigen::MatrixXd ee_jacobian(6, model.nv);
        pinocchio::computeFrameJacobian(model, data, q, ee_frame_id, ee_jacobian);

        std::cout << "Jacobian at end-effector:\n" << ee_jacobian << "\n";

        pinocchio::computeCollisions(model, data, collision_model, collision_data, q);

        int collision_count = 0;
        for (size_t k = 0; k < collision_model.collisionPairs.size(); ++k) {
            const auto& cp = collision_model.collisionPairs[k];
            const auto& cr = collision_data.collisionResults[k];

            if (cr.isCollision()) {
                ++collision_count;
                const auto& body1 = collision_model.geometryObjects[cp.first].name;
                const auto& body2 = collision_model.geometryObjects[cp.second].name;
                std::cout << "Collision detected between " << body1 << " and " << body2 << "\n";
            }
        }

        if (collision_count == 0) {
            std::cout << "No collision detected.\n";
        }
    }

    // Class members
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::string package_name_;
    std::string package_path_;
    std::string urdf_file_path_;
    std::string ee_frame_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotModelAnalyzer>());
    rclcpp::shutdown();
    return 0;
}
