#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "GoToPoiActionSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  GoToPoiActionSkill stateMachine("GoToPoiAction");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

