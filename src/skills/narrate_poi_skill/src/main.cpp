#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "NarratePoiSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  NarratePoiSkill stateMachine("NarratePoi");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "NarratePoiSkill successfully closed" << std::endl;
  return ret;
}

