#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "SayFollowMeSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  SayFollowMeSkill stateMachine("SayFollowMe");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

