#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "AlarmSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  AlarmSkill stateMachine("Alarm");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

