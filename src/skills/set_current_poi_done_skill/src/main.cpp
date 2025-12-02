#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "SetCurrentPoiDoneSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  SetCurrentPoiDoneSkill stateMachine("SetCurrentPoiDone");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

