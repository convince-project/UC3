#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "CheckIfStartSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  CheckIfStartSkill stateMachine("CheckIfStart");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "CheckIfStartSkill successfully closed" << std::endl;
  return ret;
}

