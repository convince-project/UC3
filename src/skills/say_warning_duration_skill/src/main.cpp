#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "SayWarningDurationSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  SayWarningDurationSkill stateMachine("SayWarningDuration");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "SayWarningDurationSkill successfully closed" << std::endl;
  return ret;
}

