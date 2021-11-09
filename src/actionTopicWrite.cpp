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
}

execution ActionTopicWrite::execute(const TestRepetitions& testrepetition)
{
	auto publisher = this->create_publisher<std_msgs::msg::String>(topic_, 10);
	auto message = std_msgs::msg::String();
	message.data = data_;
	publisher->publish(message);
	return execution::continueexecution;
}
