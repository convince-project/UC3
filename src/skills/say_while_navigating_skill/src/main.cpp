#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "SayWhileNavigatingSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  SayWhileNavigatingSkill stateMachine("SayWhileNavigating");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "SayWhileNavigatingSkill successfully closed" << std::endl;
  return ret;
}

