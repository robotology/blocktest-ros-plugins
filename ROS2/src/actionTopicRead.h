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

#include <condition_variable>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>

#include "action.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace BlockTestCore;

namespace RosAction
{
class ActionTopicRead : public Action, public rclcpp::Node
{
   public:
	ActionTopicRead(const CommandAttributes& commandAttributes, const std::string& testCode);
	execution execute(const TestRepetitions& testrepetition) override;
	void beforeExecute() override;
	~ActionTopicRead();

   protected:
	std::string topic_{""};
	std::string expected_{""};
	mutable bool received_{false};

	void callbackRcv1(const std_msgs::msg::String::ConstSharedPtr msg);
	void callbackRcv2(const geometry_msgs::msg::Twist::ConstSharedPtr msg);

	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_std_msgs_String_{nullptr};
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_geometry_msgs_Twist_{nullptr};

	rclcpp::executors::MultiThreadedExecutor executor_;

	ACTIONREGISTER_DEC_TYPE(ActionTopicRead)
};

}  // namespace RosAction
