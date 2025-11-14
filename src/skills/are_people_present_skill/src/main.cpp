#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "ArePeoplePresentSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  ArePeoplePresentSkill stateMachine("ArePeoplePresent");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "ArePeoplePresentSkill successfully closed" << std::endl;
  return ret;
}

