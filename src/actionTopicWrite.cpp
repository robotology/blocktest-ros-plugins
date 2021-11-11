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

#include "rosActionDepotStart.h"
#include "syntax.h"

using namespace RosAction;

ACTIONREGISTER_DEF_TYPE(RosAction::ActionTopicWrite, rosactions::rostopicwrite);

ActionTopicWrite::ActionTopicWrite(const CommandAttributes& commandAttributes, const std::string& testCode) : Action(commandAttributes, testCode), Node("rostopicwrite")
{
}

void ActionTopicWrite::beforeExecute()
{
	getCommandAttribute(rossyntax::topic, topic_);
	getCommandAttribute(rossyntax::data, data_);
	getCommandAttribute(rossyntax::datatype, dataType_);
}

execution ActionTopicWrite::execute(const TestRepetitions& testrepetition)
{
	using base = std::shared_ptr<rclcpp::PublisherBase>;

	if (dataType_ == rossyntax::dataString)
	{
		auto publisher = create_publisher<std_msgs::msg::String>(topic_, 10);
		auto message = std_msgs::msg::String();
		message.data = data_;
		publisher->publish(message);
	}
	else if (dataType_ == rossyntax::dataTypeGeometryTwist)
	{
		auto publisher = create_publisher<geometry_msgs::msg::Twist>(topic_, 10);
		auto message = geometry_msgs::msg::Twist();
		message.angular.z = 1;
		message.linear.x = 2;
		publisher->publish(message);
	}

	/*
		using derived = std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>>;
		auto publisher = create_publisher<std_msgs::msg::String>(topic_, 10);

		derived tmp = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::String>>(publisher);
		auto message = std_msgs::msg::String();
		message.data = data_;
		tmp->publish(message);*/
	return execution::continueexecution;
}