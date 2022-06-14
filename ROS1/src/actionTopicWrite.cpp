/******************************************************************************
 *                                                                            *
 * Copyright (C) 2022 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author Luca Tricerri <luca.tricerri@iit.it>
 */

#include "actionTopicWrite.h"

#include "geometry_msgs/Twist.h"
#include "json.hpp"
#include "ros1ActionDepotStart.h"
#include "std_msgs/String.h"
#include "syntax.h"

using namespace RosAction;
using json = nlohmann::json;

ACTIONREGISTER_DEF_TYPE(RosAction::ActionTopicWrite, rosactions::rostopicwrite);

ActionTopicWrite::ActionTopicWrite(const CommandAttributes& commandAttributes, const std::string& testCode) : Action(commandAttributes, testCode)  //, Node(Ros1ActionDepotStart::generateNodeName())
{
}

void ActionTopicWrite::beforeExecute()
{
	getCommandAttribute(rossyntax::topic, topic_);
	getCommandAttribute(rossyntax::data, data_);
}


execution ActionTopicWrite::execute(const TestRepetitions&)
{
	json j;
	try
	{
		j = json::parse(data_);

		if (j.contains(rossyntax::dataString))
		{
			publisherString_ = nodeHandler_.advertise<std_msgs::String>(topic_, 1000);
			std::string tmpData = j.at(rossyntax::dataString).value("data", "xxx");
			std_msgs::String msg;
			msg.data = tmpData.c_str();
			publisherString_.publish(msg);
			ros::spinOnce();
		}
		else if (j.contains(rossyntax::dataTypeGeometryTwist))
		{
			publisherTwist_ = nodeHandler_.advertise<geometry_msgs::Twist>(topic_, 1000);
			float x = j.at(rossyntax::dataTypeGeometryTwist).value("x", 0);
			float y = j.at(rossyntax::dataTypeGeometryTwist).value("y", 0);
			float z = j.at(rossyntax::dataTypeGeometryTwist).value("z", 0);
			float xa = j.at(rossyntax::dataTypeGeometryTwist).value("xa", 0);
			float ya = j.at(rossyntax::dataTypeGeometryTwist).value("ya", 0);
			float za = j.at(rossyntax::dataTypeGeometryTwist).value("za", 0);

			geometry_msgs::Twist msg;
			msg.angular.x = xa;
			msg.angular.y = ya;
			msg.angular.z = za;
			msg.linear.x = x;
			msg.linear.y = y;
			msg.linear.z = z;
			publisherTwist_.publish(msg);
			ros::spinOnce();
		}
	}
	catch (json::parse_error& e)
	{
		TXLOG(Severity::error) << "Parsing json:" << e.what() << " data:" << data_ << std::endl;
		return execution::continueexecution;
	}
	return execution::continueexecution;
}