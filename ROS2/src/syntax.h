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
namespace rossyntax
{
constexpr char data[] = {"data"};
constexpr char topic[] = {"topic"};
constexpr char datatype[] = {"datatype"};
constexpr char expected[] = {"expected"};
constexpr char tolerance[] = {"tolerance"};
constexpr char receiveTimeout[] = {"receivertimeout"};
constexpr char robometryjson[] = {"robometryjson"};
constexpr char dimensions[] = {"dimensions"};

constexpr char dataTypeGeometryTwist[] = "geometry_msgs_Twist";
constexpr char dataTypeString[] = "std_msg_String";
constexpr char dataTypeJointState[] = "sensor_msgs_JointState";
constexpr char dataTypeFloat64MultiArray[] = "std_msgs_Float64MultiArray";

};	// namespace rossyntax

namespace rosactions
{
constexpr char rostopicwrite[] = {"rostopicwrite"};
constexpr char rostopicread[] = {"rostopicread"};
constexpr char rostopicrobometry[] = {"rostopicrobometry"};
};	// namespace rosactions
