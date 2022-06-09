/******************************************************************************
 *                                                                            *
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author Luca Tricerri <luca.tricerri@iit.it>
 */

#include "actionTopicWrite.h"

#include <geometry_msgs/msg/twist.hpp>
#include "std_msgs/msg/string.hpp"
#include "json.hpp"

#include "rosActionDepotStart.h"
#include "syntax.h"

using namespace RosAction;
using json = nlohmann::json;

ACTIONREGISTER_DEF_TYPE(RosAction::ActionTopicWrite, rosactions::rostopicwrite);

ActionTopicWrite::ActionTopicWrite(const CommandAttributes& commandAttributes, const std::string& testCode) : Action(commandAttributes, testCode), Node(RosActionDepotStart::generateNodeName())
{
}

void ActionTopicWrite::beforeExecute()
{
	getCommandAttribute(rossyntax::topic, topic_);
	getCommandAttribute(rossyntax::data, data_);
}

execution ActionTopicWrite::execute(const TestRepetitions& )
{
	//using base = std::shared_ptr<rclcpp::PublisherBase>;
	json j;
	try
	{
		j = json::parse(data_);

		if (j.contains(rossyntax::dataString))
		{
			auto publisher = create_publisher<std_msgs::msg::String>(topic_, 10);
			auto message = std_msgs::msg::String();
			std::string tmpData = j.at(rossyntax::dataString).value("data","xxx");
			message.data = tmpData;
			publisher->publish(message);
			publisher.reset();
		}
		else if (j.contains(rossyntax::dataTypeGeometryTwist))
		{
			auto publisher = create_publisher<geometry_msgs::msg::Twist>(topic_, 10);
			auto message = geometry_msgs::msg::Twist();
			float x = j.at(rossyntax::dataTypeGeometryTwist).value("x",0);
			float y = j.at(rossyntax::dataTypeGeometryTwist).value("y",0);
			float z = j.at(rossyntax::dataTypeGeometryTwist).value("z",0);
			float xa = j.at(rossyntax::dataTypeGeometryTwist).value("xa",0);
			float ya = j.at(rossyntax::dataTypeGeometryTwist).value("ya",0);
			float za = j.at(rossyntax::dataTypeGeometryTwist).value("za",0);

			message.angular.x = xa;
			message.angular.y = ya;
			message.angular.z = za;
			message.linear.x = x;
			message.linear.y = y;
			message.linear.z = z;
			publisher->publish(message);
			publisher.reset();
		}
	}
	catch (json::parse_error& e)
	{
		TXLOG(Severity::error) << "Parsing json:"<<e.what()<<" data:" << data_ << std::endl;
		return execution::continueexecution;
	}
	return execution::continueexecution;
}