#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "IsTimerDoneSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  IsTimerDoneSkill stateMachine("IsTimerDone");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

