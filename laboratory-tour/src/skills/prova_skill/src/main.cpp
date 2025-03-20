#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "ProvaSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  ProvaSkill stateMachine("Prova");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  return ret;
}

