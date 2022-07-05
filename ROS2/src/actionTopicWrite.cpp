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
	if (!addNode_)
	{
		std::shared_ptr<rclcpp::Node> parent = shared_from_this();
		executor_.add_node(parent);
		addNode_ = true;
		json j = json::parse(data_);
		if (j.contains(rossyntax::dataTypeGeometryTwist))
			publisherTwist_ = create_publisher<geometry_msgs::msg::Twist>(topic_, 10);
		else if (j.contains(rossyntax::dataTypeString))
			publisherString_ = create_publisher<std_msgs::msg::String>(topic_, 10);
		else if (j.contains(rossyntax::dataTypeJointState))
			publisherJointState_ = create_publisher<sensor_msgs::msg::JointState>(topic_, 10);
		else if (j.contains(rossyntax::dataTypeFloat64MultiArray))
			publisherMultiArray_ = create_publisher<std_msgs::msg::Float64MultiArray>(topic_, 10);
	}
}

execution ActionTopicWrite::execute(const TestRepetitions&)
{
	json j;
	try
	{
		j = json::parse(data_);

		if (j.contains(rossyntax::dataTypeString))
		{
			auto message = std_msgs::msg::String();
			std::string tmpData = j.at(rossyntax::dataTypeString).value("data", "xxx");
			message.data = tmpData;
			publisherString_->publish(message);
			TXLOG(Severity::debug) << "Publish string:" << tmpData << " topic:" << topic_ << std::endl;
		}
		else if (j.contains(rossyntax::dataTypeGeometryTwist))
		{
			auto message = geometry_msgs::msg::Twist();
			float x = j.at(rossyntax::dataTypeGeometryTwist).value("x", 0);
			float y = j.at(rossyntax::dataTypeGeometryTwist).value("y", 0);
			float z = j.at(rossyntax::dataTypeGeometryTwist).value("z", 0);
			float xa = j.at(rossyntax::dataTypeGeometryTwist).value("xa", 0);
			float ya = j.at(rossyntax::dataTypeGeometryTwist).value("ya", 0);
			float za = j.at(rossyntax::dataTypeGeometryTwist).value("za", 0);

			message.angular.x = xa;
			message.angular.y = ya;
			message.angular.z = za;
			message.linear.x = x;
			message.linear.y = y;
			message.linear.z = z;
			publisherTwist_->publish(message);
			TXLOG(Severity::debug) << "Publish twist"
								   << " topic:" << topic_ << std::endl;
		}
		else if (j.contains(rossyntax::dataTypeJointState))
		{
			auto message = sensor_msgs::msg::JointState();
			std::string name = j.at(rossyntax::dataTypeJointState).value("name", "xxx");
			double position = j.at(rossyntax::dataTypeJointState).value("position", 0);
			double velocity = j.at(rossyntax::dataTypeJointState).value("velocity", 0);
			double effort = j.at(rossyntax::dataTypeJointState).value("effort", 0);

			message.name.push_back(name);
			message.position.push_back(position);
			message.velocity.push_back(velocity);
			message.effort.push_back(effort);
			publisherJointState_->publish(message);
			TXLOG(Severity::debug) << "Publish jointstate"
								   << " topic:" << topic_ << std::endl;
		}
		else if (j.contains(rossyntax::dataTypeFloat64MultiArray))
		{
			auto message = std_msgs::msg::Float64MultiArray();

			auto out=j.at(rossyntax::dataTypeFloat64MultiArray).at("list").get<std::vector<double>>();
			message.data.insert(message.data.end(),out.begin(),out.end());

			publisherMultiArray_->publish(message);
			TXLOG(Severity::debug) << "Publish Float64MultiArray"
								   << " topic:" << topic_ << std::endl;
		}
	}
	catch (json::parse_error& e)
	{
		TXLOG(Severity::error) << "Parsing json:" << e.what() << " data:" << data_ << std::endl;
		return execution::continueexecution;
	}

	return execution::continueexecution;
}
