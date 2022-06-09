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
#include <thread>
#include <chrono>
using namespace std::chrono_literals;



#include "json.hpp"
#include "rosActionDepotStart.h"
#include "syntax.h"

using std::placeholders::_1;
using namespace std::chrono_literals;
using json = nlohmann::json;
using namespace RosAction;

ACTIONREGISTER_DEF_TYPE(RosAction::ActionTopicRead, rosactions::rostopicread);

ActionTopicRead::ActionTopicRead(const CommandAttributes& commandAttributes, const std::string& testCode) : Action(commandAttributes, testCode), Node(RosActionDepotStart::generateNodeName())
{
}

void ActionTopicRead::beforeExecute()
{
	exec_.add_node(shared_from_this());

	getCommandAttribute(rossyntax::topic, topic_);
	getCommandAttribute(rossyntax::expected, expected_);

	try
	{
		json j = json::parse(expected_);

		if (j.contains(rossyntax::dataString))
		{
			if (subscription_std_msgs_String_ == nullptr)
			{
				subscription_std_msgs_String_ = this->create_subscription<std_msgs::msg::String>(topic_, 10, std::bind(&ActionTopicRead::callbackRcv1, this, _1));
				TXLOG(Severity::debug) << "Subscribe to msg type:string" << std::endl;
			}
		}
		else if (j.contains(rossyntax::dataTypeGeometryTwist))
		{
			if (subscription_geometry_msgs_Twist_ == nullptr)
			{
				subscription_geometry_msgs_Twist_ = this->create_subscription<geometry_msgs::msg::Twist>(topic_, 10, std::bind(&ActionTopicRead::callbackRcv2, this, _1));
				TXLOG(Severity::debug) << "Subscribe to msg type:geometry_twist" << std::endl;
			}
		}
		else
		{
			TXLOG(Severity::error) << "Not supported data type" << std::endl;
		}
	}
	catch (json::parse_error& e)
	{
		TXLOG(Severity::error) << "Parsing json:" << e.what() << " expected:" << expected_ << std::endl;
	}
}

execution ActionTopicRead::execute(const TestRepetitions& testrepetition)
{
	for (int t = 0; t < 10; ++t)
	{
		_exec.spin_once(shared_from_this());
		std::this_thread::sleep_for(100ms);
		if (received_)
			break;
	}

	if (!received_)
	{
		std::stringstream logStream;
		logStream << "Event not received: "
				  << " topic:" << topic_;
		addProblem(testrepetition, Severity::error, logStream.str(), true);
	}
	received_ = false;
	return execution::continueexecution;
}

void ActionTopicRead::callbackRcv1(const std_msgs::msg::String::ConstSharedPtr msg) const
{
	json j = json::parse(expected_);
	std::string expectedData = j.at(rossyntax::dataString).value("data", "xxx");

	received_ = true;
	if (msg->data != expectedData)
	{
		std::stringstream logStream;
		logStream << "Read unexpected value: " << msg->data << "expected:" << expectedData << " topic:" << topic_;
		addProblem({0, 0}, Severity::error, logStream.str(), true);
	}
	TXLOG(Severity::debug) << "Callback receive std_msgs::msg::String:" << msg->data << std::endl;
}

void ActionTopicRead::callbackRcv2(const geometry_msgs::msg::Twist::ConstSharedPtr msg) const
{
	json j = json::parse(expected_);
	float x = j.at(rossyntax::dataTypeGeometryTwist).value("x", 0);
	float y = j.at(rossyntax::dataTypeGeometryTwist).value("y", 0);
	float z = j.at(rossyntax::dataTypeGeometryTwist).value("z", 0);
	float xa = j.at(rossyntax::dataTypeGeometryTwist).value("xa", 0);
	float ya = j.at(rossyntax::dataTypeGeometryTwist).value("ya", 0);
	float za = j.at(rossyntax::dataTypeGeometryTwist).value("za", 0);

	received_ = true;

	if (msg->angular.x != xa || msg->angular.y != ya || msg->angular.z != za || msg->linear.x != x || msg->linear.y != y || msg->linear.z != z)
	{
		std::stringstream logStream;
		logStream << "Read unexpected value: "
				  << " topic:" << topic_;
		addProblem({0, 0}, Severity::error, logStream.str(), false);
	}

	TXLOG(Severity::debug) << "Callback receive geometry_msgs::msg::Twist:" << std::endl;
}
