#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>


#include <iostream>
#include "AlarmBatteryLowSkill.h"

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  AlarmBatteryLowSkill stateMachine("AlarmBatteryLow");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
  
}

