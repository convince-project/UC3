#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "SetPoi2Skill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  SetPoi2Skill stateMachine("SetPoi2");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

