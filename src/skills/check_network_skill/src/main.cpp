#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "CheckNetworkSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  CheckNetworkSkill stateMachine("CheckNetwork");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "CheckNetworkSkill successfully closed" << std::endl;
  return ret;
}

