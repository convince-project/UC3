#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "HardwareFaultSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  HardwareFaultSkill stateMachine("HardwareFault");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

