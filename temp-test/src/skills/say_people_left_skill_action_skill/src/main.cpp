#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "SayPeopleLeftSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  SayPeopleLeftSkill stateMachine("SayPeopleLeft");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

