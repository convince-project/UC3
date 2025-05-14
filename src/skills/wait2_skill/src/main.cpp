#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>


#include <iostream>
#include "WaitSkill.h"

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  WaitSkill stateMachine("Wait2");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
  
}

