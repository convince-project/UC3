#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>

#include <iostream>
#include "CheckNetworkSkill.h"

#include <thread>
#include <chrono>

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  CheckNetworkSkill stateMachine("CheckNetwork");
  stateMachine.start(argc, argv);

  int ret=app.exec();

  return ret;

}

