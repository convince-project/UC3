/*
 * SPDX-FileCopyrightText: 2024-2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <vector>

#include <cstdio>
#include <cmath>
#include <mutex>

#include <iostream>
#include <fstream>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include <tinyxml.h>

using namespace yarp::os;
using namespace std;

#define RETURN_CODE_ERROR 1
#define RETURN_CODE_OK    0

void print_help()
{
    std::cout << "Welcome to YarpDeviceParamParserGenerator tool. Syntax:\n";
    std::cout << "1) yarpDeviceParamParserGenerator --class_name \"className\" --module_name \"moduleName\" --input_filename_md \"filename.md\" [--input_extra_comments \"comments.md\"] [--generate_md] [--generate_ini] [--generate_yarpdev] [--generate_yarprobotinterface] [--generate_all] [--output_dir \"output_path\"] [--debug_mode]\n";
    std::cout << "or:\n";
    std::cout << "2) yarpDeviceParamParserGenerator --class_name \"className\" --module_name \"moduleName\" --input_filename_ini \"filename.ini\" [--input_extra_comments \"comments.md\"] [--generate_md] [--generate_ini] [--generate_yarpdev] [--generate_yarprobotinterface] [--generate_all] [--output_dir \"output_path\"] [--debug_mode]\n";
}

int main(int argc, char *argv[])
{
    bool debug_mode = false;
    std::string input_filename = "in.scxml";
    std::string output1_filename = "out1.cpp";
    std::string output2_filename = "out2.cpp";


#if 0 //debug only!
   std::cout << "Invocation command " << argc << ":";
   for (size_t i = 0; i < argc; ++i)
   {
        std::cout << argv[i] << " ";
   }
   std::cout << "<EOL>"<<std::endl;
#endif

   if (argc == 1)
   {
       print_help();
       return RETURN_CODE_ERROR;
   }

   for (int i = 1; i < argc; ++i)
   {
        std::string arg = argv[i];
        if (arg == "--help") {
            print_help();
            return RETURN_CODE_ERROR;
        }
        else if (arg == "--input_filename"  && i+1 < argc && argv[i+1][0] != '-') {
            input_filename = argv[i+1];
            i++;
        }
        else if (arg == "--output1_filename" && i + 1 < argc && argv[i + 1][0] != '-') {
            output1_filename = argv[i + 1];
            i++;
        }
        else if (arg == "--debug_mode") {
            debug_mode = true;
        }
    }

   //open output file
   std::ofstream file_out1(output1_filename);
   bool b = file_out1.is_open();
   if (!b) { std::cerr << "Failed to write file:" << output1_filename; return RETURN_CODE_ERROR; }

   TiXmlDocument doc;
   doc.LoadFile(input_filename.c_str());

   TiXmlElement* titleElement = doc.FirstChildElement("scxml");
   if (titleElement)
   {
       for (TiXmlElement* e = titleElement->FirstChildElement("state"); e != NULL; e = e->NextSiblingElement("state"))
       {
           auto* state_id_c = e->Attribute("id");
           std::string state_id;
           if (state_id_c == nullptr) {std::cout << "error, missing id attribute"; continue;}
           state_id = std::string(state_id_c);
           std::cout << state_id << endl;
           for (TiXmlElement* e2 = e->FirstChildElement("transition"); e2 != NULL; e2 = e2->NextSiblingElement("transition"))
           {
               auto* transition_event_c = e2->Attribute("event");
               auto* transition_target_c = e2->Attribute("target");
               std::string transition_event;
               std::string transition_target;
               if (transition_event_c) {transition_event = std::string(transition_event_c);} else {}
               if (transition_target_c) {transition_target = std::string(transition_target_c);} else {}
               std::cout << "    " << state_id << " " << transition_event << " " << transition_target << endl;

               file_out1 << "    " << state_id << " " << transition_event << " " << transition_target << endl;
           }
           std::cout << "--" << endl;
       }
   }
   else
   {
       std::cout << "Failed.";
       file_out1.close();
       return RETURN_CODE_ERROR;
   }

   //close outputfile
   file_out1.close();

   std::cout << "Generation process successfully completed.";
   return RETURN_CODE_OK;
}
