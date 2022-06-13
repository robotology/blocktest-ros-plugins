/******************************************************************************
 *                                                                            *
 * Copyright (C) 2022 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author Luca Tricerri <luca.tricerri@iit.it>
 */

#pragma once

#include "actionDepotStart.h"


using namespace BlockTestCore;


using std::placeholders::_1;

class Ros1ActionDepotStart :public ActionDepotStart
{
    public:
        Ros1ActionDepotStart(); 
        ~Ros1ActionDepotStart() override;

        void configure(const std::map<std::string,std::string>&) override;
        void stop() override;

        static std::string generateNodeName();

};
