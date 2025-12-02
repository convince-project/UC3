#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "PeopleLeftSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  PeopleLeftSkill stateMachine("PeopleLeft");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "PeopleLeftSkill successfully closed" << std::endl;
  return ret;
}

