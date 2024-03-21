#include "BatteryLevelSkillDataModel.h"
#include <QDebug>


bool BatteryLevelSkillDataModel::setup(const QVariantMap& initialDataValues)
{
    return true;
}


void BatteryLevelSkillDataModel::log(std::string to_log) {
    qInfo(to_log.c_str());
}