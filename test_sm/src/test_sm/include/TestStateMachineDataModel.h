/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#pragma once

#include <QScxmlCppDataModel>
#include <string>


class TestStateMachineDataModel: public QScxmlCppDataModel
{
    Q_SCXML_DATAMODEL

public:
    TestStateMachineDataModel() = default;
    bool setup(const QVariantMap& initialDataValues) override;
    void log(std::string to_log);
private: 
};

Q_DECLARE_METATYPE(::TestStateMachineDataModel*)
