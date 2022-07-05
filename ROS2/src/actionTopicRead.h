/******************************************************************************
 *                                                                            *
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author Luca Tricerri <luca.tricerri@iit.it>
 */

#pragma once

#include <atomic>
#include <condition_variable>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "action.h"
#include "rclcpp/rclcpp.hpp"

using namespace BlockTestCore;

namespace RosAction
{
class ActionTopicRead : public Action, public rclcpp::Node
{
   public:
	ActionTopicRead(const CommandAttributes& commandAttributes, const std::string& testCode);
	execution execute(const TestRepetitions& testrepetition) override;
	void beforeExecute() override;
	~ActionTopicRead() override;

   protected:
	std::string topic_{""};
	std::string expected_{""};
	int receiveTimeout_{1000};
	std::unique_ptr<std::thread> threadTimeout_;
	void timeout();
	std::atomic<bool> received_{false};
	bool addNode_{false};

	virtual void callbackRcvString(const std_msgs::msg::String::ConstSharedPtr msg);
	virtual void callbackRcvTwist(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
	virtual void callbackRcvJointState(const sensor_msgs::msg::JointState::ConstSharedPtr msg);
	virtual void callbackRcvMultiArray(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg);

	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_std_msgs_String_{nullptr};
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_geometry_msgs_Twist_{nullptr};
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_msgs_JointState_{nullptr};
	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_msgs_MultyArray_{nullptr};

	rclcpp::executors::MultiThreadedExecutor executor_;

	ACTIONREGISTER_DEC_TYPE(ActionTopicRead)
};

}  // namespace RosAction
