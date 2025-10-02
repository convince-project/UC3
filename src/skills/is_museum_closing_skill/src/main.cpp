#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "IsMuseumClosingSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  IsMuseumClosingSkill stateMachine("IsMuseumClosing");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "IsMuseumClosingSkill successfully closed" << std::endl;
  return ret;
}

