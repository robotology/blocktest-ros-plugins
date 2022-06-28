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
		getCommandAttribute(rossyntax::receiveTimeout, receiveTimeout_);
		BufferConfig bufferConfig;
		// In case of using different message type, we use the json configuration
		bufferConfig.yarp_robot_name = "robot";
		bufferConfig.description_list = { "" };
		// FIXME the dimensionality of the message is not handled, the test is sending scalar values
		bufferConfig.channels = { {"name",{1,1}}, {"position",{1,1}}, {"velocity",{1,1}}, {"effort",{1,1}} };
		bufferConfig.filename = "robometry_blocktest_data";
		bufferConfig.n_samples = 100000;
		bufferConfig.save_period = 120.0;
		bufferConfig.data_threshold = 300;
		bufferConfig.save_periodically = true;
		bufferConfig.enable_compression = true;
		bool ok = m_bufferManager.configure(bufferConfig);
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
	if(msg.get() != nullptr)
	{
		TXLOG(Severity::debug) << "I am reading something......" << std::endl;
		TXLOG(Severity::debug) << "Sizes: name: " << msg->name.size() << " position: " << msg->position.size()<< " velocity: " << msg->velocity.size() << " effort: " << msg->effort.size() << std::endl;
		m_bufferManager.push_back(msg->name, "name");
		m_bufferManager.push_back(msg->position, "position");
		m_bufferManager.push_back(msg->velocity, "velocity");
		m_bufferManager.push_back(msg->effort, "effort");
		// TODO maybe we have to handle in this way
		// for (size_t t = 0; t < msg->name.size(); ++t)
		// {
		// 	TXLOG(Severity::debug) << "name: " << msg->name[t] << " position: " << msg->position[t] << " velocity: " << msg->velocity[t] << " effort: " << msg->effort[t] << std::endl;
		// 	m_bufferManager.push_back({msg->get().name}, "name");
		// }
		received_ = true;
	}
}



