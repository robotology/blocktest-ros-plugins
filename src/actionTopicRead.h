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

   protected:
	std::string topic_{""};
	std::string expected_{""};
	mutable bool received_{false};

	void callbackRcv1(const std_msgs::msg::String::ConstSharedPtr msg) const;
	void callbackRcv2(const geometry_msgs::msg::Twist::ConstSharedPtr msg) const;

	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_std_msgs_String_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_geometry_msgs_Twist_;

	ACTIONREGISTER_DEC_TYPE(ActionTopicRead)
};

}  // namespace RosAction
