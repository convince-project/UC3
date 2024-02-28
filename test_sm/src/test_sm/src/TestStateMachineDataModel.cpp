/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
#include "TestStateMachineDataModel.h"
#include <iostream>
#include<QDebug>


bool TestStateMachineDataModel::setup(const QVariantMap &initialDataValues)
{
    std::cout << "init" << std::endl;
    return true;
}



void TestStateMachineDataModel::log(std::string to_log) {
    std::cout << to_log << std::endl;
}