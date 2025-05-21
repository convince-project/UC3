#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "StopServiceSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  StopServiceSkill stateMachine("StopService");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

