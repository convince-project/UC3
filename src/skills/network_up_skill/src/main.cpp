#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "NetworkUpSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  NetworkUpSkill stateMachine("NetworkUp");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "NetworkUpSkill successfully closed" << std::endl;
  return ret;
}

