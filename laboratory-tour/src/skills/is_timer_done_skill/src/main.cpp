#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>


#include <iostream>
#include "IsTimerDoneSkill.h"

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  IsTimerDoneSkill stateMachine("IsTimerDone");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
  
}

