#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>


#include <iostream>
#include "CheckIfStartSkill.h"

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  CheckIfStartSkill stateMachine("CheckIfStart2");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
  
}

