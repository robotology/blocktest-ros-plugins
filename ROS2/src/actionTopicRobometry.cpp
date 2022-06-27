/******************************************************************************
 *                                                                            *
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author Luca Tricerri <luca.tricerri@iit.it>
 */
#include "actionTopicRobometry.h"

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "json.hpp"
#include "rosActionDepotStart.h"
#include "syntax.h"

using std::placeholders::_1;
using namespace RosAction;
using namespace robometry;


ACTIONREGISTER_DEF_TYPE(RosAction::ActionTopicRobometry, rosactions::rostopicrobometry);

ActionTopicRobometry::ActionTopicRobometry(const CommandAttributes& commandAttributes, const std::string& testCode) : ActionTopicRead(commandAttributes, testCode)
{
}

void ActionTopicRobometry::beforeExecute()
{
	received_ = false;
	if (!addNode_)
	{
		std::shared_ptr<rclcpp::Node> parent = shared_from_this();
		executor_.add_node(parent);
		addNode_ = true;

		getCommandAttribute(rossyntax::topic, topic_);
		getCommandAttribute(rossyntax::robometryjson, robometryJson_);
		getCommandAttribute(rossyntax::receiveTimeout, receiveTimeout_);
		BufferConfig bufferConfig;
		bool ok = bufferConfigFromJson(bufferConfig, robometryJson_);
		ok = ok && m_bufferManager.configure(bufferConfig);
		if (!ok)
		{
			TXLOG(Severity::error) << "Failed to configure robometry::BufferManager" << std::endl;
			return;
		}

		if (subscription_msgs_JointState_ == nullptr)
		{
			subscription_msgs_JointState_ = this->create_subscription<sensor_msgs::msg::JointState>(topic_, 10, std::bind(&ActionTopicRobometry::callbackRcvJointState, this, _1));
			TXLOG(Severity::debug) << "Subscribe to msg type:joint_state" << std::endl;
		}
	}
}

void ActionTopicRobometry::callbackRcvJointState(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
	TXLOG(Severity::debug) << "I am reading something......" << std::endl;
}



