#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "SetPoiSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  SetPoiSkill stateMachine("SetPoi");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

