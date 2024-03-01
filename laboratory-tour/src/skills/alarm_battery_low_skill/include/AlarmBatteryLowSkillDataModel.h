/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#pragma once

#include <QScxmlCppDataModel>
#include <QVariant>
#include <string>


class AlarmBatteryLowSkillDataModel: public QScxmlCppDataModel
{
    Q_SCXML_DATAMODEL

public:
    AlarmBatteryLowSkillDataModel() = default;
    bool setup(const QVariantMap& initialDataValues) override;
    void log(std::string to_log);
private: 
    QVariant response;
};

Q_DECLARE_METATYPE(::AlarmBatteryLowSkillDataModel*)
