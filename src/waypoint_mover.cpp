#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h> 
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class WaypointMover : public rclcpp::Node {
public:
    WaypointMover() : Node("waypoint_mover") {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
        execution_thread_ = std::thread([this]() {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            moveit_routine();
        });
    }

    ~WaypointMover() {
        if (execution_thread_.joinable()) execution_thread_.join();
    }

private:
    std::thread execution_thread_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    visualization_msgs::msg::MarkerArray marker_array_;

    struct Waypoint {
        std::string name;
        geometry_msgs::msg::Pose pose;
    };

    void moveit_routine() {
        auto move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "ur_manipulator");
        move_group.setMaxVelocityScalingFactor(0.5);
        move_group.setMaxAccelerationScalingFactor(0.5);

        spawn_table();
        std::vector<Waypoint> waypoints = load_waypoints();

        if (waypoints.empty()) return;
        init_markers(waypoints);
        for (size_t i = 0; i < waypoints.size(); ++i) {
            update_marker_color(i, 1.0, 1.0, 0.0, 1.0); 
            move_to_pose(move_group, waypoints[i].pose, waypoints[i].name);
            update_marker_color(i, 0.0, 1.0, 0.0, 1.0);
        }
    }
    void init_markers(const std::vector<Waypoint>& waypoints) {
        marker_array_.markers.clear();
        
        for (size_t i = 0; i < waypoints.size(); ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = this->now();
            marker.ns = "waypoints";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose = waypoints[i].pose;
            marker.scale.x = 0.05; 
            marker.scale.y = 0.05; 
            marker.scale.z = 0.05;

            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0; 

            marker_array_.markers.push_back(marker);
        }
        marker_pub_->publish(marker_array_);
    }

    void update_marker_color(int id, float r, float g, float b, float a) {
        if (id >= 0 && id < (int)marker_array_.markers.size()) {
            marker_array_.markers[id].color.r = r;
            marker_array_.markers[id].color.g = g;
            marker_array_.markers[id].color.b = b;
            marker_array_.markers[id].color.a = a;
            marker_array_.markers[id].header.stamp = this->now(); 
            marker_pub_->publish(marker_array_);
        }
    }

    std::vector<Waypoint> load_waypoints() {
        std::vector<Waypoint> loaded_waypoints;
        std::vector<std::string> waypoint_names;
        
        this->declare_parameter("waypoint_names", std::vector<std::string>());
        if (!this->get_parameter("waypoint_names", waypoint_names) || waypoint_names.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No waypoints provided in config");
            return loaded_waypoints;
        }

        for (const auto& name : waypoint_names) {
            std::vector<double> pose_data;
            this->declare_parameter(name, std::vector<double>());
            
            if (this->get_parameter(name, pose_data) && pose_data.size() == 7) {
                Waypoint wp;
                wp.name = name;
                wp.pose.position.x = pose_data[0];
                wp.pose.position.y = pose_data[1];
                wp.pose.position.z = pose_data[2];
                wp.pose.orientation.x = pose_data[3];
                wp.pose.orientation.y = pose_data[4];
                wp.pose.orientation.z = pose_data[5];
                wp.pose.orientation.w = pose_data[6];
                loaded_waypoints.push_back(wp);
            }
        }
        return loaded_waypoints;
    }

    void move_to_pose(moveit::planning_interface::MoveGroupInterface& move_group, 
                      const geometry_msgs::msg::Pose& target_pose, 
                      const std::string& waypoint_name) {
        
        RCLCPP_INFO(this->get_logger(), "Moving to waypoint: %s", waypoint_name.c_str());

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

        if (fraction >= 1.0) {
            RCLCPP_INFO(this->get_logger(), ">>CARTESIAN path found, executing the path");
            move_group.execute(trajectory);
        } 
        else {
            // Strategy B: Joint
            RCLCPP_WARN(this->get_logger(), "Cartesian path failed. moving in joint space", fraction * 100.0);
            move_group.setPoseTarget(target_pose);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            
            if (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                move_group.execute(my_plan);
            } else {
                RCLCPP_ERROR(this->get_logger(), "FAILED to reach %s", waypoint_name.c_str());
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    void spawn_table() {
        moveit::planning_interface::PlanningSceneInterface psi;
        moveit_msgs::msg::CollisionObject co;
        co.header.frame_id = "world"; 
        co.id = "table";
        shape_msgs::msg::SolidPrimitive prim;
        prim.type = prim.BOX;
        prim.dimensions = {1.0, 1.0, 0.2};
        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0; 
        pose.position.z = -0.2;
        co.primitives.push_back(prim);
        co.primitive_poses.push_back(pose);
        co.operation = co.ADD;
        psi.applyCollisionObjects({co});
        std::this_thread::sleep_for(std::chrono::seconds(1)); 
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointMover>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}