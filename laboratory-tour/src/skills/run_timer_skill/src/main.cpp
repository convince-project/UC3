#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "RunTimerSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  RunTimerSkill stateMachine("RunTimer");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}
