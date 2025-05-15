#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "WaitSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  WaitSkill stateMachine("Wait");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

