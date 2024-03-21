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


class BatteryLevelSkillDataModel: public QScxmlCppDataModel
{
    Q_SCXML_DATAMODEL

public:
    BatteryLevelSkillDataModel() = default;
    bool setup(const QVariantMap& initialDataValues) override;
    void log(std::string to_log);
private: 
    double m_level;
};

Q_DECLARE_METATYPE(::BatteryLevelSkillDataModel*)
