#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "IsPoiDone2Skill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  IsPoiDone2Skill stateMachine("IsPoiDone2");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

