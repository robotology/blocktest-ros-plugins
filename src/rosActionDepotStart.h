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

#include "actionDepotStart.h"


using namespace BlockTestCore;

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class RosActionDepotStart :public ActionDepotStart
{
    public:
        RosActionDepotStart(); 
        ~RosActionDepotStart() override;

        void configure(const std::map<std::string,std::string>&) override;
        void stop() override;

};
