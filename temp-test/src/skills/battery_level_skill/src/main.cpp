#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "BatteryLevelSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  BatteryLevelSkill stateMachine("BatteryLevel");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

