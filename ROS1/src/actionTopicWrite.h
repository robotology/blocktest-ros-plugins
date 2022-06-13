/******************************************************************************
 *                                                                            *
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author Luca Tricerri <luca.tricerri@iit.it>
 */

#pragma once

#include "action.h"
//#include "rclcpp/rclcpp.hpp"

using namespace BlockTestCore;

namespace RosAction
{
class ActionTopicWrite : public Action//, public ros::Node
{
   public:
	ActionTopicWrite(const CommandAttributes& commandAttributes, const std::string& testCode);
	execution execute(const TestRepetitions& testrepetition) override;
	void beforeExecute() override;

   protected:
	std::string topic_{""};
	std::string data_{""};

	ACTIONREGISTER_DEC_TYPE(ActionTopicWrite)
};

}  // namespace RosAction