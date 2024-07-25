#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "SayDurationWarningSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  SayDurationWarningSkill stateMachine("SayDurationWarning");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

