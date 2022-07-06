/******************************************************************************
 *                                                                            *
 * Copyright (C) 2022 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author Luca Tricerri <luca.tricerri@iit.it>
 */
#include "actionTopicRead.h"

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "json.hpp"
#include "rosActionDepotStart.h"
#include "syntax.h"

using std::placeholders::_1;
using namespace std::chrono_literals;
using json = nlohmann::json;
using namespace RosAction;
using namespace std::chrono_literals;

ACTIONREGISTER_DEF_TYPE(RosAction::ActionTopicRead, rosactions::rostopicread);

ActionTopicRead::ActionTopicRead(const CommandAttributes& commandAttributes, const std::string& testCode) : Action(commandAttributes, testCode), Node(RosActionDepotStart::generateNodeName())
{
}

void ActionTopicRead::beforeExecute()
{
	received_ = false;

	if (!addNode_)
	{
		std::shared_ptr<rclcpp::Node> parent = shared_from_this();
		executor_.add_node(parent);
		addNode_ = true;

		getCommandAttribute(rossyntax::topic, topic_);
		getCommandAttribute(rossyntax::expected, expected_);
		getCommandAttribute(rossyntax::receiveTimeout, receiveTimeout_);
		getCommandAttribute(rossyntax::tolerance, tolerance_);

		try
		{
			json j = json::parse(expected_);

			if (j.contains(rossyntax::dataTypeString))
			{
				if (subscription_std_msgs_String_ == nullptr)
				{
					subscription_std_msgs_String_ = this->create_subscription<std_msgs::msg::String>(topic_, 10, std::bind(&ActionTopicRead::callbackRcvString, this, _1));
					TXLOG(Severity::debug) << "Subscribe to msg type:string" << std::endl;
				}
			}
			else if (j.contains(rossyntax::dataTypeGeometryTwist))
			{
				if (subscription_geometry_msgs_Twist_ == nullptr)
				{
					subscription_geometry_msgs_Twist_ = this->create_subscription<geometry_msgs::msg::Twist>(topic_, 10, std::bind(&ActionTopicRead::callbackRcvTwist, this, _1));
					TXLOG(Severity::debug) << "Subscribe to msg type:geometry_twist" << std::endl;
				}
			}
			else if (j.contains(rossyntax::dataTypeJointState))
			{
				if (subscription_msgs_JointState_ == nullptr)
				{
					subscription_msgs_JointState_ = this->create_subscription<sensor_msgs::msg::JointState>(topic_, 10, std::bind(&ActionTopicRead::callbackRcvJointState, this, _1));
					TXLOG(Severity::debug) << "Subscribe to msg type:joint_state" << std::endl;
				}
			}
			else if (j.contains(rossyntax::dataTypeFloat64MultiArray))
			{
				if (subscription_msgs_MultyArray_ == nullptr)
				{
					subscription_msgs_MultyArray_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(topic_, 10, std::bind(&ActionTopicRead::callbackRcvMultiArray, this, _1));
					TXLOG(Severity::debug) << "Subscribe to msg type:multyarray" << std::endl;
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
}

execution ActionTopicRead::execute(const TestRepetitions& testrepetition)
{
	threadTimeout_ = std::make_unique<std::thread>(&ActionTopicRead::timeout, this);

	executor_.spin();
	if (received_)
	{
		TXLOG(Severity::debug) << "Received msg" << std::endl;
	}
	else
	{
		std::stringstream logStream;
		logStream << "Event not received: "
				  << " topic:" << topic_;
		addProblem(testrepetition, Severity::error, logStream.str(), true);
	}
	threadTimeout_->join();
	return execution::continueexecution;
}

void ActionTopicRead::callbackRcvString(const std_msgs::msg::String::ConstSharedPtr msg)
{
	json j = json::parse(expected_);
	std::string expectedData = j.at(rossyntax::dataTypeString).value("data", "xxx");

	received_ = true;
	if (msg->data != expectedData)
	{
		std::stringstream logStream;
		logStream << "Read unexpected value: " << msg->data << "expected:" << expectedData << " topic:" << topic_;
		addProblem({0, 0}, Severity::error, logStream.str(), true);
	}
	TXLOG(Severity::debug) << "Callback receive std_msgs::msg::String:" << msg->data << std::endl;
	executor_.cancel();
}

void ActionTopicRead::callbackRcvTwist(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
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
	executor_.cancel();
}

void ActionTopicRead::callbackRcvJointState(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
	json j = json::parse(expected_);
	std::string name = j.at(rossyntax::dataTypeJointState).value("name", "xxx");
	double position = j.at(rossyntax::dataTypeJointState).at("position").get<nlohmann::json::number_float_t>();
	double velocity = j.at(rossyntax::dataTypeJointState).at("velocity").get<nlohmann::json::number_float_t>();
	double effort =  j.at(rossyntax::dataTypeJointState).at("effort").get<nlohmann::json::number_float_t>();

	bool flag = false;
	for (size_t t = 0; t < msg->name.size(); ++t)
	{
		if (msg->name[t] == name)
		{
			flag = true;
			if (std::abs(msg->position[t] - position) > tolerance_)
			{
				std::stringstream logStream;
				logStream << "Read unexpected value for joint: "<<name
						  << " topic:" << topic_ << " position:" << msg->position[t] << " expected:" << position << " tolerance:" << tolerance_;
				addProblem({0, 0}, Severity::error, logStream.str(), true);
			}
			/*
			if (std::abs(msg->velocity[t] - velocity) > tolerance_)
			{
				std::stringstream logStream;
				logStream << "Read unexpected value for velocity: "
						  << " topic:" << topic_ << " velocity:" << msg->velocity[t] << " expected:" << velocity << " tolerance:" << tolerance_;
				addProblem({0, 0}, Severity::error, logStream.str(), true);
			}
			if (std::abs(msg->effort[t] - effort) > tolerance_)
			{
				std::stringstream logStream;
				logStream << "Read unexpected value for effort: "
						  << " topic:" << topic_ << " effort:" << msg->effort[t] << " expected:" << effort << " tolerance:" << tolerance_;
				addProblem({0, 0}, Severity::error, logStream.str(), true);
			}
			*/
			break;
		}
	}

	if (!flag)
	{
		std::stringstream logStream;
		logStream << "Joint name not found in msg: "
				  << " topic:" << topic_ << " name:" << name;
		addProblem({0, 0}, Severity::error, logStream.str(), false);
	}

	received_ = true;
	TXLOG(Severity::debug) << "Callback receive sensor_msgs::msg::JointState:" << std::endl;
	executor_.cancel();
}

void ActionTopicRead::callbackRcvMultiArray(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
{
	json j = json::parse(expected_);
	auto expected = j.at(rossyntax::dataTypeFloat64MultiArray).at("list").get<std::vector<double>>();

	if (expected != msg->data)
	{
		std::stringstream logStream;
		logStream << "Read unexpected value: "
				  << " topic:" << topic_;
		addProblem({0, 0}, Severity::error, logStream.str(), false);
	}

	received_ = true;
	TXLOG(Severity::debug) << "Callback receive std_msgs::msg::Float64MultiArray" << std::endl;
	executor_.cancel();
}

ActionTopicRead::~ActionTopicRead()
{
	subscription_std_msgs_String_ = nullptr;
	subscription_geometry_msgs_Twist_ = nullptr;
	subscription_msgs_JointState_ = nullptr;
}

void ActionTopicRead::timeout()
{
	auto starttime = std::chrono::system_clock::now();
	while (!received_)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		auto currenttime = std::chrono::system_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currenttime - starttime).count();
		if (elapsed > receiveTimeout_)
			break;
	}
	executor_.cancel();
}