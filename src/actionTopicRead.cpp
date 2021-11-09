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
	TXLOG(Severity::info) << "Topic" << topic_ << std::endl;
}

execution ActionTopicRead::execute(const TestRepetitions& testrepetition)
{
	// subscription_ = this->create_subscription<std_msgs::msg::String>(topic_, 10, std::bind(&ActionTopicRead::callbackRcv, this, _1));
	subscription_ = this->create_subscription<std_msgs::msg::String>(topic_, 10, [&](const std_msgs::msg::String::ConstSharedPtr msg){
		TXLOG(Severity::info) << "Rcv" << msg->data << std::endl;});
	std::unique_lock<std::mutex> lk(myMutex_);
	cv_.wait_for(lk, 10000ms);
	return execution::continueexecution;
}

void ActionTopicRead::callbackRcv(const std_msgs::msg::String::ConstSharedPtr msg) const
{
	TXLOG(Severity::info) << "Rcv" << msg->data << std::endl;
	if (msg->data != expected_)
	{
		std::stringstream logStream;
		logStream << "Read unexpected value: " << msg->data << " topic:" << topic_;
		addProblem({0, 0}, Severity::error, logStream.str(), true);
	}
	cv_.notify_all();
}
