#include "BatteryDrainerSkillDataModel.h"
#include <QDebug>


bool BatteryDrainerSkillDataModel::setup(const QVariantMap& initialDataValues)
{
    return true;
}


void BatteryDrainerSkillDataModel::log(std::string to_log) {
    qInfo(to_log.c_str());
}