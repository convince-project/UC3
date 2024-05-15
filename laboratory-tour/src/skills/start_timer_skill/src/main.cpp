#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>


#include <iostream>
#include "StartTimerSkill.h"

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  StartTimerSkill stateMachine("StartTimer");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
  
}

