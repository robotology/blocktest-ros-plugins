/******************************************************************************
 *                                                                            *
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author Luca Tricerri <luca.tricerri@iit.it>
 */

#include "ros1ActionDepotStart.h"

#include <map>
#include <string>

#include "action.h"
#include "general.h"
#include "logger.h"
#include <ros/ros.h>

ACTIONDEPOTSTART(Ros1ActionDepotStart)

Ros1ActionDepotStart::Ros1ActionDepotStart()
{
	int tmp;
	ros::init(tmp, nullptr,"Blocktest_Node");
	TXLOG(Severity::info) << "Library setup" << std::endl;
}

void Ros1ActionDepotStart::configure(const std::map<std::string, std::string> &)
{
	TXLOG(Severity::info) << "Library config called:" << std::endl;
}

void Ros1ActionDepotStart::stop()
{
	TXLOG(Severity::info) << "Library stop called:" << std::endl;
	ros::shutdown();
}

Ros1ActionDepotStart::~Ros1ActionDepotStart()
{
}

std::string Ros1ActionDepotStart::generateNodeName()
{
	static int count=0;
	std::string out="MyNode"+std::to_string(count);
	count++;
	return out;
}
