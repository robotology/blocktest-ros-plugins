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

#include <robometry/BufferManager.h>

#include "actionTopicRead.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace BlockTestCore;
namespace RosAction
{
class ActionTopicRobometry : public ActionTopicRead
{
   public:
	ActionTopicRobometry(const CommandAttributes& commandAttributes, const std::string& testCode);
	void beforeExecute() override;
	~ActionTopicRobometry() = default;

   protected:

	//virtual void callbackRcvString(const std_msgs::msg::String::ConstSharedPtr msg);
	//virtual void callbackRcvTwist(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
	void callbackRcvJointState(const sensor_msgs::msg::JointState::ConstSharedPtr msg) override;
	std::string robometryJson_;
	robometry::BufferManager m_bufferManager;
	ACTIONREGISTER_DEC_TYPE(ActionTopicRobometry)
};

}  // namespace RosAction
