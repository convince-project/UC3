#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "SayDurationExceededSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  SayDurationExceededSkill stateMachine("SayDurationExceeded");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "SayDurationExceededSkill successfully closed" << std::endl;
  return ret;
}

