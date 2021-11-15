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
namespace rossyntax
{
constexpr char data[] = {"data"};
constexpr char topic[] = {"topic"};
constexpr char datatype[] = {"datatype"};
constexpr char expected[] = {"expected"};

constexpr char dataTypeGeometryTwist[] = "geometry_msgs_Twist";
constexpr char dataString[] = "std_msg_String";
};	// namespace rossyntax

namespace rosactions
{
constexpr char rostopicwrite[] = {"rostopicwrite"};
constexpr char rostopicread[] = {"rostopicread"};
};	// namespace rosactions
