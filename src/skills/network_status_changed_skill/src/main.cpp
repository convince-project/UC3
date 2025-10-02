#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "NetworkStatusChangedSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  NetworkStatusChangedSkill stateMachine("NetworkStatusChanged");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "NetworkStatusChangedSkill successfully closed" << std::endl;
  return ret;
}

