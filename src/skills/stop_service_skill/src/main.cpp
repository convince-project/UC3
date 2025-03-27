#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>


#include <iostream>
#include "StopServiceSkill.h"

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  StopServiceSkill stateMachine("StopService");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
  
}

