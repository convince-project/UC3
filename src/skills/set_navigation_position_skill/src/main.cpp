#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "SetNavigationPositionSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  SetNavigationPositionSkill stateMachine("SetNavigationPosition");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

