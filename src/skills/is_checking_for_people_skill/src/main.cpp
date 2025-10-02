#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "IsCheckingForPeopleSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  IsCheckingForPeopleSkill stateMachine("IsCheckingForPeople");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "IsCheckingForPeopleSkill successfully closed" << std::endl;
  return ret;
}

