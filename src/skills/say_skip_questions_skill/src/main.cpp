#include <QCoreApplication>
#include <QScxmlStateMachine>
#include <QDebug>
#include <iostream>
#include <thread>
#include <chrono>
#include "SaySkipQuestionsSkill.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  SaySkipQuestionsSkill stateMachine("SaySkipQuestions");
  stateMachine.start(argc, argv);

  int ret=app.exec();
  
  std::cout << "SaySkipQuestionsSkill successfully closed" << std::endl;
  return ret;
}

