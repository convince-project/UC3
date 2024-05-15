#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>


#include <iostream>
#include "NotifyChargedSkill.h"

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  NotifyChargedSkill stateMachine("NotifyCharged");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
  
}

