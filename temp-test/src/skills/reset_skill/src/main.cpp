#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "ResetSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  ResetSkill stateMachine("Reset");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

