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

#include <mutex>
#include <condition_variable>

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

	void callbackRcv(const std_msgs::msg::String::ConstSharedPtr msg) const;

	mutable std::mutex myMutex_;
	mutable std::condition_variable cv_;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

	ACTIONREGISTER_DEC_TYPE(ActionTopicRead)
};

}  // namespace RosAction
