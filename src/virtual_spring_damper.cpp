#include <memory>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "ros_gz_interfaces/msg/entity_wrench.hpp"

using namespace std::chrono_literals;

class VirtualSpringDamper : public rclcpp::Node {
    public:
        VirtualSpringDamper() : Node("virtual_spring_damper") {
            // Parameter definition
            this->declare_parameter("undeformed_length", 0.3);
            this->declare_parameter("stiffness", 1000.0);
            this->declare_parameter("damping", 10.0);
            // Read parameters
            undeformed_length_ = this->get_parameter("undeformed_length").as_double();
            stiffness_ = this->get_parameter("stiffness").as_double();
            damping_ = this->get_parameter("damping").as_double();
            // TF2 definitions
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            // Wrench publisher
            wrench_pub_ = this->create_publisher<ros_gz_interfaces::msg::EntityWrench>("/world/ic2d_empty/wrench", 10);
            // Force marker publisher
            marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/world/ic2d_empty/wrench/markers", 10);
            // Timer
            timer_ = this->create_wall_timer(20ms, std::bind(&VirtualSpringDamper::on_timer, this));
        }
    private:
        void on_timer() {
            geometry_msgs::msg::TransformStamped tf_stamped;
            try {
                tf_stamped = tf_buffer_->lookupTransform("link_1", "link_2", tf2::TimePointZero);
                if(previous_tf_.header.stamp.sec != 0) { // Check if there is a valid previous transform
                    // Compute total force (spring + damper)
                    double force = compute_spring_force(tf_stamped.transform, undeformed_length_, stiffness_) + compute_damping_force(previous_tf_, tf_stamped, damping_);
                    // Build the wrench message
                    auto msg = ros_gz_interfaces::msg::EntityWrench();
                    msg.header.stamp = tf_stamped.header.stamp;
                    msg.header.frame_id = tf_stamped.header.frame_id;
                    msg.entity.id = 14; // link_1
                    msg.wrench.force.x = force;
                    // Publish the message (link_1)
                    wrench_pub_->publish(msg);
                    // Switch to link_2
                    msg.entity.id = 17; // link_2
                    msg.wrench.force.x = -force;
                    // Publish the message (link_2)
                    wrench_pub_->publish(msg);
                    // Publish the arrow markers
                    publish_arrow_markers(tf_stamped, force);
                }
                // Update previous transform
                previous_tf_ = tf_stamped;
            } catch(tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
            }
        }
        double compute_spring_force(const geometry_msgs::msg::Transform & transform, double undeformed_length, double stiffness) {
            return (transform.translation.x - undeformed_length) * stiffness;
        }
        double compute_damping_force(const geometry_msgs::msg::TransformStamped & previous, const geometry_msgs::msg::TransformStamped & current, double damping) {
            // Compute time difference
            double dt = (current.header.stamp.sec - previous.header.stamp.sec) + (current.header.stamp.nanosec - previous.header.stamp.nanosec) / 1e9;
            // Compute relative speed
            double speed = (current.transform.translation.x - previous.transform.translation.x) / dt;
            // Compute damping force
            return speed * damping;
        }
        void publish_arrow_markers(const geometry_msgs::msg::TransformStamped & tf_stamped, double force) {
            visualization_msgs::msg::MarkerArray marker_array;
            // Marker for link_1
            visualization_msgs::msg::Marker marker_1;
            marker_1.header.stamp = tf_stamped.header.stamp;
            marker_1.header.frame_id = "link_1";
            marker_1.ns = "force_arrows";
            marker_1.id = 0;
            marker_1.type = visualization_msgs::msg::Marker::ARROW;
            marker_1.action = visualization_msgs::msg::Marker::ADD;
            marker_1.scale.x = force/5000;
            marker_1.scale.y = 0.02;
            marker_1.scale.z = 0.02;
            marker_1.color.r = 0.0;
            marker_1.color.g = 1.0;
            marker_1.color.b = 0.0;
            marker_1.color.a = 1.0;
            // Add marker to array
            marker_array.markers.push_back(marker_1);
            // Marker for link_2
            visualization_msgs::msg::Marker marker_2;
            marker_2.header.stamp = tf_stamped.header.stamp;
            marker_2.header.frame_id = "link_2";
            marker_2.ns = "force_arrows";
            marker_2.id = 1;
            marker_2.type = visualization_msgs::msg::Marker::ARROW;
            marker_2.action = visualization_msgs::msg::Marker::ADD;
            marker_2.scale.x = -force/5000;
            marker_2.scale.y = 0.02;
            marker_2.scale.z = 0.02;
            marker_2.color.r = 0.0;
            marker_2.color.g = 1.0;
            marker_2.color.b = 0.0;
            marker_2.color.a = 1.0;
            // Add marker to array
            marker_array.markers.push_back(marker_2);
            // Publish marker
            marker_pub_->publish(marker_array);
        }
        double undeformed_length_, stiffness_, damping_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::Publisher<ros_gz_interfaces::msg::EntityWrench>::SharedPtr wrench_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        geometry_msgs::msg::TransformStamped previous_tf_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualSpringDamper>());
    rclcpp::shutdown();
    return 0;
}