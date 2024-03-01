/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
#include "AlarmBatteryLowSkillDataModel.h"
#include <iostream>
#include<QDebug>


bool AlarmBatteryLowSkillDataModel::setup(const QVariantMap& initialDataValues)
{
    std::cout << "init" << std::endl;
    return true;
}



void AlarmBatteryLowSkillDataModel::log(std::string to_log) {
    std::cout << to_log << std::endl;
}