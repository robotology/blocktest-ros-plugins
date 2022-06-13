/******************************************************************************
 *                                                                            *
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author Luca Tricerri <luca.tricerri@iit.it>
 */

#include "rosActionDepotStart.h"

#include <map>
#include <string>

#include "action.h"
#include "general.h"
#include "logger.h"
#include <ros/ros.h>

ACTIONDEPOTSTART(RosActionDepotStart)

RosActionDepotStart::RosActionDepotStart()
{
	//ros::init(0, nullptr);
	TXLOG(Severity::info) << "Library setup" << std::endl;
}

void RosActionDepotStart::configure(const std::map<std::string, std::string> &)
{
	TXLOG(Severity::info) << "Library config called:" << std::endl;
}

void RosActionDepotStart::stop()
{
	TXLOG(Severity::info) << "Library stop called:" << std::endl;
	//rclcpp::shutdown();
}

RosActionDepotStart::~RosActionDepotStart()
{
}

std::string RosActionDepotStart::generateNodeName()
{
	static int count=0;
	std::string out="MyNode"+std::to_string(count);
	count++;
	return out;
}
