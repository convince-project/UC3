#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "IsAtChargingStationSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  IsAtChargingStationSkill stateMachine("IsAtChargingStation");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

