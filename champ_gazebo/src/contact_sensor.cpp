/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
// Modified by Ravi Kumar Thakur

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <champ/utils/urdf_loader.h>
#include <gz/transport/Node.hh>
#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/contact.pb.h>
#include <champ_msgs/msg/contacts_stamped.hpp>
#include <signal.h>

class ContactSensor : public rclcpp::Node
{
public:
    ContactSensor() :
        foot_contacts_{false, false, false, false},
        Node("contacts_sensor", 
            rclcpp::NodeOptions()
                .allow_undeclared_parameters(true)
                .automatically_declare_parameters_from_overrides(true))
    {
        initializeFootLinks();
        initializePublisher();
        initializeGazeboSubscriber();
    }

    void publishContacts()    
    {
        auto contacts_msg = champ_msgs::msg::ContactsStamped();
        contacts_msg.header.stamp = this->get_clock()->now();
        contacts_msg.contacts.resize(4);
        
        for(size_t i = 0; i < 4; i++)
        {
            contacts_msg.contacts[i] = foot_contacts_[i];
        }
        
        contacts_publisher_->publish(contacts_msg);
    }

private:
    void initializeFootLinks()
    {
        std::vector<std::string> joint_names = 
            champ::URDF::getLinkNames(this->get_node_parameters_interface());
        
        // Store the foot link names (assumed to be at specific indices)
        foot_links_.push_back(joint_names[2]);
        foot_links_.push_back(joint_names[6]);
        foot_links_.push_back(joint_names[10]);
        foot_links_.push_back(joint_names[14]);

        // Declare and get world name parameter
        this->declare_parameter("world_name", "default");
        world_name_ = this->get_parameter("world_name").as_string();

        // Print foot link names for debugging
        RCLCPP_INFO(this->get_logger(), "Monitoring contacts for foot links:");
        for (const auto& link : foot_links_) {
            RCLCPP_INFO(this->get_logger(), "  - %s", link.c_str());
        }
    }

    void initializePublisher()
    {
        contacts_publisher_ = this->create_publisher<champ_msgs::msg::ContactsStamped>(
            "foot_contacts", 
            10
        );
    }

    void initializeGazeboSubscriber()
    {
        gz_node_ = std::make_shared<gz::transport::Node>();
        
        // Subscribe to the contacts topic for the specific world
        std::string topic = "/world/" + world_name_ + "/physics/contacts";
        
        if (!gz_node_->Subscribe(topic, &ContactSensor::gazeboCallback_, this))
        {
            RCLCPP_ERROR(this->get_logger(), "Error subscribing to topic [%s]", topic.c_str());
            throw std::runtime_error("Failed to subscribe to Gazebo contacts topic");
        }
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to Gazebo contacts on topic: %s", topic.c_str());
    }

    void gazeboCallback_(const gz::msgs::Contacts &_msg)
    {
        // Reset all contacts
        std::fill(foot_contacts_, foot_contacts_ + 4, false);

        // Process each contact
        for (int i = 0; i < _msg.contact_size(); ++i) 
        {
            const auto& contact = _msg.contact(i);
            
            // Extract collision names directly from the entities
            std::string collision1_name = contact.collision1().name();
            std::string collision2_name = contact.collision2().name();

            // Check if either collision involves our foot links
            for(size_t j = 0; j < 4; j++)
            {
                // Check if the collision name contains our foot link name
                if (collision1_name.find(foot_links_[j]) != std::string::npos ||
                    collision2_name.find(foot_links_[j]) != std::string::npos)
                {
                    foot_contacts_[j] = true;
                    RCLCPP_DEBUG(this->get_logger(), "Contact detected for foot %zu: %s", 
                                j, foot_links_[j].c_str());
                    break;
                }
            }
        }
    }

    bool foot_contacts_[4];
    std::vector<std::string> foot_links_;
    std::string world_name_;
    rclcpp::Publisher<champ_msgs::msg::ContactsStamped>::SharedPtr contacts_publisher_;
    std::shared_ptr<gz::transport::Node> gz_node_;
};

void exitHandler(int sig)
{
    rclcpp::shutdown();
}

int main(int argc, char **argv)
{
    // Setup signal handling
    signal(SIGINT, exitHandler);
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ContactSensor>();
    
    rclcpp::Rate loop_rate(50);  // 50 Hz update rate
    
    while (rclcpp::ok())
    {
        node->publishContacts();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}