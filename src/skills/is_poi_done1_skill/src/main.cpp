#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "IsPoiDone1Skill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  IsPoiDone1Skill stateMachine("IsPoiDone1");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

