/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
#include "TestStateMachineDataModel.h"
#include <iostream>



bool TestStateMachineDataModel::setup(const QVariantMap &initialDataValues)
{
    return true;
}



void TestStateMachineDataModel::log(std::string to_log) {
    std::cout << to_log;
}