#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "StopAndTurnBackSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  StopAndTurnBackSkill stateMachine("StopAndTurnBack");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

