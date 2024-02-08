#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>


#include <iostream>
#include "BatteryDrainerSkill.h"

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  BatteryDrainerSkill stateMachine("BatteryDrainer");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
  
}

