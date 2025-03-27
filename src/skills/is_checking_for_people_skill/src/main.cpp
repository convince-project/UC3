#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>


#include <iostream>
#include "IsCheckingForPeopleSkill.h"

#include <thread>
#include <chrono>



int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  IsCheckingForPeopleSkill stateMachine("IsCheckingForPeople");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
  
}

