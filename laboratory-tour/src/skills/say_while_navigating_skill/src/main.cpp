#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>


#include <iostream>
#include "SayWhileNavigatingSkill.h"

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  SayWhileNavigatingSkill stateMachine("SayWhileNavigating");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
  
}

