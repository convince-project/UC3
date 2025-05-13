#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "SetNotTurningSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  SetNotTurningSkill stateMachine("SetNotTurning");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

