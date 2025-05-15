#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "StopTourTimerSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  StopTourTimerSkill stateMachine("StopTourTimer");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

