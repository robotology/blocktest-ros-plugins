/******************************************************************************
 *                                                                            *
 * Copyright (C) 2022 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author Nicolo Genesio <nicolo.genesio@iit.it>
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
using json = nlohmann::json;

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
		getCommandAttribute(rossyntax::dimensions, dimensions_str);

		json j = json::parse(dimensions_str);
	    dimensions = j.at("list").get<std::vector<size_t>>();
		if (dimensions.size() > 2) {
			TXLOG(Severity::error) << "Dimensions invalid, it must be >0, <= 2" << std::endl;
			return;
		}

		BufferConfig bufferConfig;
		// In case of using different message type, we use the json configuration
		bufferConfig.yarp_robot_name = "robot";
		bufferConfig.description_list = {""};
		// FIXME the dimensionality of the message is not handled, the test is sending scalar values
		bufferConfig.channels = {{"name", dimensions}, {"position", dimensions}, {"velocity", dimensions}, {"effort", dimensions}};
		bufferConfig.path = "./";
		bufferConfig.filename = "robometry_blocktest_data";
		bufferConfig.n_samples = 100000;
		bufferConfig.save_period = 120.0;
		bufferConfig.data_threshold = 300;
		bufferConfig.save_periodically = true;
		bufferConfig.enable_compression = true;
		bufferConfig.auto_save = true;	// FIXME The destructor is not invoked for some reason
		bool ok = bufferManager_.configure(bufferConfig);
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
	if (msg.get() != nullptr)
	{
		if (msg->name.size() != dimensions[0]*dimensions[1]) {
			TXLOG(Severity::error) << "Msg size and robometry dimension mismatch" << std::endl;
			executor_.cancel();
			return;
		}

		bufferManager_.push_back(msg->name, "name");
		bufferManager_.push_back(msg->position, "position");
		bufferManager_.push_back(msg->velocity, "velocity");
		bufferManager_.push_back(msg->effort, "effort");
		received_ = true;
	}
	executor_.cancel();
}

void ActionTopicRobometry::afterExecuteAllRepetitions()
{
	bufferManager_.saveToFile();
}
