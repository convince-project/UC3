#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "StartServiceSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  StartServiceSkill stateMachine("StartService");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

