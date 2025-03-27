#include "AlarmBatteryLowSkillDataModel.h"
#include <QDebug>


bool AlarmBatteryLowSkillDataModel::setup(const QVariantMap& initialDataValues)
{
    return true;
}


void AlarmBatteryLowSkillDataModel::log(std::string to_log) {
    qInfo(to_log.c_str());
}