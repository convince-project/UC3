#include "BatteryLevelDataModel.h"
#include <QDebug>


bool BatteryLevelDataModel::setup(const QVariantMap& initialDataValues)
{
    m_level = 100.0;
    return true;
}


void BatteryLevelDataModel::log(std::string to_log) {
    qInfo(to_log.c_str());
}