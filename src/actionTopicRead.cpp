/******************************************************************************
 *                                                                            *
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author Luca Tricerri <luca.tricerri@iit.it>
 */
#include "actionTopicRead.h"

#include <functional>
#include <memory>

#include "rosActionDepotStart.h"
#include "syntax.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

using namespace RosAction;

ACTIONREGISTER_DEF_TYPE(RosAction::ActionTopicRead, rosactions::rostopicread);

ActionTopicRead::ActionTopicRead(const CommandAttributes& commandAttributes, const std::string& testCode) : Action(commandAttributes, testCode), Node("rostopicwrite")
{
}

void ActionTopicRead::beforeExecute()
{
	getCommandAttribute(rossyntax::topic, topic_);
	getCommandAttribute(rossyntax::expected, expected_);
	if (subscription_ == nullptr)
		subscription_ = this->create_subscription<std_msgs::msg::String>(topic_, 10, std::bind(&ActionTopicRead::callbackRcv, this, _1));
}

execution ActionTopicRead::execute(const TestRepetitions& testrepetition)
{
	rclcpp::spin_some(shared_from_this());
	if(!received_)
	{
		std::stringstream logStream;
		logStream << "Event not received: " << " topic:" << topic_;
		addProblem(testrepetition, Severity::error, logStream.str(), false);
	}
	received_=false;
	return execution::continueexecution;
}

void ActionTopicRead::callbackRcv(const std_msgs::msg::String::ConstSharedPtr msg) const
{
	received_=true;
	if (msg->data != expected_)
	{
		std::stringstream logStream;
		logStream << "Read unexpected value: " << msg->data << " topic:" << topic_;
		addProblem({0, 0}, Severity::error, logStream.str(), true);
	}
	TXLOG(Severity::debug) << "Callback receive:" << msg->data << std::endl;
}
