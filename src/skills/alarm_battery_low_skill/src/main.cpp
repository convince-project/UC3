#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "AlarmBatteryLowSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  AlarmBatteryLowSkill stateMachine("AlarmBatteryLow");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "AlarmBatteryLowSkill successfully closed" << std::endl;
  return ret;
}

