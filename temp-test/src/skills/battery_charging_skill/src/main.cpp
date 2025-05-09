#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "BatteryChargingSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  BatteryChargingSkill stateMachine("BatteryCharging");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

