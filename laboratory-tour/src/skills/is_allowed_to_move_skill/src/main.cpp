#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>


#include <iostream>
#include "IsAllowedToMoveSkill.h"

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  IsAllowedToMoveSkill stateMachine("IsAllowedToMove");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
  
}

