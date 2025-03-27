#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "IsPoiDoneSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  IsPoiDoneSkill stateMachine("IsPoiDone");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

