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
  
  return ret;
}

