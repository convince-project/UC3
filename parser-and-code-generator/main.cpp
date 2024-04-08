
/*
 * SPDX-FileCopyrightText: 2024-2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tinyxml2.h"
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <map>
#include <vector>

#define modelFilePath "./specification/examples/museum-guide/main-XML/full-model.xml"
#define interfaceFilePath "./specification/examples/museum-guide/interface-definition-IDL/interfaces.xml"

#define RETURN_CODE_ERROR 1
#define RETURN_CODE_OK    0

#define cmdTick "CMD_TICK"
#define cmdHalt "CMD_HALT"
#define rspTick "TICK_RESPONSE"
#define rspHalt "HALT_RESPONSE"

static std::map<std::string, bool> eventsMap;
static std::string modelFileName       = modelFilePath;
static std::string interfaceFileName   = interfaceFilePath;

struct skillDataStr{
    std::string SMName;
    std::string className;
    std::string skillName;
    std::string skillType;
};

struct eventDataStr{
    std::string event;
    std::string eventType;
    std::string target;
    std::map<std::string, std::string> paramMap;
    std::string paramExpr;
    std::string componentName;
    std::string functionName;
    std::string eventName;
    std::string interfaceName;
    std::string interfaceType;
    std::string interfaceDataType;
    std::string interfaceDataField;
};

struct cppCodeStr
{
    std::string includeCode; 
    std::string converterCode; 
    std::string constructorCode;
    std::string spinCode;
    std::string startCode;
    std::string servicesCode; 
    std::string handlersCode;
    std::string endstartCode;
    std::string callbacksCode; 
};

struct hCodeStr
{
    std::string includeCode;
    std::string statusCode;
    std::string openClassCode;
    std::string publicCode;
    std::string privateCode;
    std::string closeClassCode;
};

struct cppDataModelCodeStr
{
    std::string includeCode; 
    std::string spinCode;
    std::string logCode;
    std::string setupCode;
    std::string subscriptionCode; 
    std::string callbacksCode; 
    std::string endsetupCode;
};

struct topicCodeStr
{
    std::string includeCode;
    std::string publicCode;
    std::string privateCode;
    std::string servicesCode;
    std::string callbacksCode;
};

/* Get info from strings*/
void getDataFromEvent(eventDataStr& eventData) 
{
    std::string firstWord, secondWord, thirdWord;
    std::string event = eventData.event;
    if (event == ""){
        std::cerr << "Event has no value" << std::endl;
        return;
    }
    size_t firstDotPos = event.find('.');
    if (firstDotPos != std::string::npos) {
        firstWord = event.substr(0, firstDotPos);
        size_t secondDotPos = event.find('.', firstDotPos + 1);
        if (secondDotPos != std::string::npos) {
            secondWord = event.substr(firstDotPos+1, secondDotPos - firstDotPos - 1);
            thirdWord = event.substr(secondDotPos + 1);
        }
        else{
            std::cerr << "Error in name format for event: "<< event << std::endl;
        }
    }
    else{
        std::cerr << "Error in name format for event: "<< event << std::endl;
    }
    eventData.componentName = firstWord;
    eventData.functionName   = secondWord;
    eventData.eventName     = thirdWord;
}

void turnToSnakeCase(const std::string input, std::string& output)
{   
    std::string temp = input;
    bool flag = false;
    for (char c : temp){
        if (isupper(c)){
            if(flag){
                output.push_back('_');
            }
            else{
                flag = true;
            }
        }
        output.push_back(tolower(c));
    }
}

void getDataTypePath(const std::string input, std::string& output)
{
    //e.g. from "sensor_msgs::msg::BatteryState" to "sensor_msgs/msg/battery_state"
    std::string temp = input;
    size_t pos = 0;
    if (temp == ""){
        std::cerr << "Input DataType has no value" << std::endl;
        return;
    }

    while ((pos = temp.find("::", pos)) != std::string::npos) {
        temp.replace(pos, 2, "/");
        pos += 1;
    }

    turnToSnakeCase(temp, output);

}

bool getDataFromRootName(const std::string attributeName, skillDataStr& skillData)
{
    if (attributeName != ""){
        std::cout << "Root attribute name: " << attributeName << std::endl;
        size_t dotPos = attributeName.find("Skill");
        if (dotPos != std::string::npos){
            skillData.SMName = attributeName;
            skillData.skillName = attributeName.substr(0, dotPos);
            skillData.className = attributeName.substr(0, dotPos + 5);
            skillData.skillType = attributeName.substr(dotPos + 5);
            if(skillData.skillType == ""){
                std::cerr << "Skill type not found" << std::endl;
                return false;
            }
        }
        else{
            std::cerr << "Skill name not found" << std::endl;
            return false;
        }
    } 
    else{
        std::cerr << "Attribute 'name' not found or has no value" << std::endl;
        return false;
    }
    return true;

}


/* Extract from XML - Get from element*/
bool getElementAttValue(tinyxml2::XMLElement* element, const std::string attribute, std::string& attributeValue)
{
    const char* value = element->Attribute(attribute.c_str());
    if (value){
        attributeValue = std::string(value);
        return true;
    }
    else{
        std::cerr << "Attribute '" << attribute << "' of element '"<< element->Value()<<"' not found or has no value" << std::endl;
        return false;
    }
}

bool getElementText(tinyxml2::XMLElement* element, std::string& textValue)
{
    const char* text = element->GetText();
    if (text){
        textValue = std::string(text);
        return true;
    }
    else{
        std::cerr << "Text of element '"<< element->Value()<<"' not found or has no value" << std::endl;
        return false;
    }
}


/* Extract from XML - Find Elements*/
bool findElementByTagAndAttValue(tinyxml2::XMLElement* root, const std::string tag, const std::string attributeName, const std::string attributeValue, tinyxml2::XMLElement*& element)
{
    for (tinyxml2::XMLElement* child = root->FirstChildElement(); child; child = child->NextSiblingElement()) {
        if (strcmp(child->Value(), tag.c_str()) == 0) {
            const char* id = child->Attribute(attributeName.c_str());
            if (id && std::string(id) == attributeValue){
                element = child;
                return true;
            }
        }
        if (findElementByTagAndAttValue(child, tag, attributeName, attributeValue, element)) {
            return true;
        }
    }

    return false;
}

bool findElementByTag(tinyxml2::XMLElement* root, const std::string tag, tinyxml2::XMLElement*& element)
{
    for (tinyxml2::XMLElement* child = root->FirstChildElement(); child; child = child->NextSiblingElement()) {
        if (strcmp(child->Value(), tag.c_str()) == 0){
            element = child;
            return true;
        }
        if (findElementByTag(child, tag, element)) {
            return true;
        }
    }

    return false;
}

void findElementVectorByTagAndAttribute(tinyxml2::XMLElement* root, const std::string tag, const std::string attribute, std::vector<tinyxml2::XMLElement*>& elementVector)
{
    for (tinyxml2::XMLElement* child = root->FirstChildElement(); child; child = child->NextSiblingElement()){
        if (strcmp(child->Value(), tag.c_str()) == 0) {
            const char* childAttribute = child->Attribute(attribute.c_str());
            if (childAttribute) {
                elementVector.push_back(child);
            }
        }
        findElementVectorByTagAndAttribute(child, tag, attribute, elementVector);
    }
}

void findElementVectorByTag(tinyxml2::XMLElement* root, const std::string tag, std::vector<tinyxml2::XMLElement*>& elementVector)
{
    for (tinyxml2::XMLElement* child = root->FirstChildElement(); child; child = child->NextSiblingElement()){
        if (strcmp(child->Value(), tag.c_str()) == 0) {
            elementVector.push_back(child);
        }
        findElementVectorByTag(child, tag, elementVector);
    }
}


/* Read from XML files */
bool extractInterfaceName(const std::string fileName, eventDataStr& eventData)
{
    const std::string componentName = eventData.componentName;
    std::string interfaceName;
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(fileName.c_str()) != tinyxml2::XML_SUCCESS) {
        std::cerr << "Failed to load '" << fileName << "' file" << std::endl;
        return false;
    }
 
    tinyxml2::XMLElement* root = doc.RootElement();
    if (!root) {
        std::cerr << "No root element found" << std::endl;
        return false;
    }
    
    tinyxml2::XMLElement* element;
    if(!findElementByTagAndAttValue(root, std::string("componentDeclaration"), std::string("id"), componentName, element)){
        std::cerr << "\nNo component '" << componentName << "'found in file '" << fileName << "'."<< std::endl;
        return false;
    }
    if (!getElementAttValue(element, std::string("interface"), interfaceName)){
        std::cerr << "\nNo interface found for component '" << componentName << "'."<< std::endl;
        return false;
    }
    eventData.interfaceName = interfaceName;

    return true;  
}

bool extractInterfaceType(const std::string fileName, eventDataStr& eventData)
{
    const std::string interfaceName = eventData.interfaceName;
    const std::string functionName = eventData.functionName;
    std::string interfaceType, interfaceDataType, interfaceDataField;
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(fileName.c_str()) != tinyxml2::XML_SUCCESS) {
        std::cerr << "Failed to load '" << fileName << "' file" << std::endl;
        return false;
    }
 
    tinyxml2::XMLElement* root = doc.RootElement();
    if (!root) {
        std::cerr << "No root element found" << std::endl;
        return false;
    }

    std::string idValue;
    tinyxml2::XMLElement* elementInterface, *elementInterfaceType, *elementFunction, *elementDataType, *elementDataField;
    if (!findElementByTagAndAttValue(root, std::string("interface"), std::string("id"), interfaceName, elementInterface))
    {
        std::cerr << "\nNo interface '" << interfaceName << "'found in file '" << fileName << "'."<< std::endl;
        return false;
    }
    if(!findElementByTagAndAttValue(elementInterface, std::string("function"), std::string("id"), functionName, elementFunction))
    {
        std::cerr << "No function '" << functionName << "'found in file '" << fileName << "'."<< std::endl;
        return false;
    }
    if(!findElementByTag(elementFunction, std::string("interface"), elementInterfaceType))
    {
        return false;
    }
    if(!getElementAttValue(elementInterfaceType, std::string("type"), interfaceType))
    {
        return false;
    }
    eventData.interfaceType = interfaceType;
    if(!findElementByTag(elementFunction, std::string("dataType"), elementDataType))
    {   
        std::cerr << "No tag <dataType> for function '" << functionName << "'."<< std::endl;
        //Not every interface has a defined DataType TODO
        return true;
    } 
    if(!getElementText(elementDataType, interfaceDataType))
    {
         std::cerr << "No value in tag <dataType> for function '" << functionName << "'."<< std::endl;
        //Not every interface has a defined DataType TODO
        return true;
    }
    if(!findElementByTag(elementFunction, std::string("dataField"), elementDataField))
    {   
        std::cerr << "No tag <dataField> for function '" << functionName << "'."<< std::endl;
        //Not every interface has a defined DataType TODO
        return true;
    } 
    if(!getElementText(elementDataField, interfaceDataField))
    {
         std::cerr << "No value in tag <dataField> for function '" << functionName << "'."<< std::endl;
        //Not every interface has a defined DataType TODO
        return true;
    }
    eventData.interfaceDataType = interfaceDataType;
    eventData.interfaceDataField = interfaceDataField;
    return true;
}


bool extractFromSCXML(tinyxml2::XMLDocument& doc, const std::string fileName, std::string& rootName, std::vector<tinyxml2::XMLElement*>& elementsTransition, std::vector<tinyxml2::XMLElement*>& elementsSend) 
{
    if (doc.LoadFile(fileName.c_str()) != tinyxml2::XML_SUCCESS) {
        std::cerr << "Failed to load '" << fileName << "' file" << std::endl;
        return false;
    }
 
    tinyxml2::XMLElement* root = doc.RootElement();
    if (!root) {
        std::cerr << "No root element found" << std::endl;
        return false;
    }
    rootName = std::string(root->Attribute("name")); 
    if(rootName == std::string(""))
    {
        std::cerr << "No root name found" << std::endl;
        return false;
    }
      
    // Get transitions
    findElementVectorByTagAndAttribute(root, std::string("transition"), "event", elementsTransition);
    if (elementsTransition.empty()) {
        std::cout << "No transition elements found." << std::endl;
    } 
    else 
    {
        std::cout << "Transition elements found." << std::endl;
    }

    // Get Send
    findElementVectorByTagAndAttribute(root, std::string("send"), "event", elementsSend);
    if (elementsSend.empty()) {
        std::cout << "No Send elements found." << std::endl;
    } 
    else 
    {
        std::cout << "Send elements found." << std::endl;
    }
    return true;
}

/* Generate code */

void printEventData(eventDataStr eventData)
{
    std::cout << "\tcomponent=" << eventData.componentName << ", service=" << eventData.functionName << ", eventName=" << eventData.eventName << std::endl;
    std::cout<< "\tinterface=" << eventData.interfaceName << ", type=" << eventData.interfaceType;
    if(eventData.interfaceDataType != "")
    {
        std::cout << ", dataType=" << eventData.interfaceDataType;
    }
    if(eventData.interfaceDataField != "")
    {
        std::cout << ", dataField=" << eventData.interfaceDataField;
    }
    std::cout << std::endl;
    
}

void printSkillData(skillDataStr skillData)
{
    std::cout << "-----------" << std::endl;
    std::cout << "Class name: " << skillData.className << std::endl << "Skill name: " << skillData.skillName << std::endl << "Skill type: " << skillData.skillType << std::endl;
    std::cout << "-----------" << std::endl;
}

void processEvent(eventDataStr eventData, const skillDataStr skillData, std::string target, hCodeStr& hCode, cppCodeStr& cppCode, topicCodeStr& topicCode)
{
    const std::string className = skillData.className;
    const std::string skillName = skillData.skillName;
    const std::string skillType = skillData.skillType;

    if(eventsMap.find(eventData.event) != eventsMap.end()){
        return;
    } 
    eventsMap[eventData.event];

    if(eventData.event == cmdTick)
    {
        hCode.publicCode += "\tvoid tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::Tick" + skillType + "::Request> request,\n"
            "\t\t\t   std::shared_ptr<bt_interfaces::srv::Tick" + skillType + "::Response>      response);\n";
    }
    else if(eventData.event == cmdHalt)
    {
        hCode.publicCode += "\tvoid halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::Halt" + skillType + "::Request> request,\n"
            "\t\t\t   [[maybe_unused]] std::shared_ptr<bt_interfaces::srv::Halt" + skillType + "::Response> response);\n";
    }
    else if(eventData.event == rspHalt)
    {
        cppCode.handlersCode += 
            "\tm_stateMachine.connectToEvent(\"" + eventData.event + "\", [this]([[maybe_unused]]const QScxmlEvent & event){\n"
            "\t\tRCLCPP_INFO(m_node->get_logger(), \""+ className +"::haltresponse\");\n"
            "\t\tm_haltResult.store(true);\n"
            "\t});\n\n";
    }
    else if(eventData.event == rspTick)
    {
        cppCode.handlersCode += 
            "\tm_stateMachine.connectToEvent(\"" + eventData.event + "\", [this]([[maybe_unused]]const QScxmlEvent & event){\n"
            "\t\tRCLCPP_INFO(m_node->get_logger(), \"" + className + "::tickReturn %s\", event.data().toMap()[\"result\"].toString().toStdString().c_str());\n"
            "\t\tstd::string result = event.data().toMap()[\"result\"].toString().toStdString();\n"
            "\t\tif (result == \"SUCCESS\" )\n"
            "\t\t{\n" 
            "\t\t\tm_tickResult.store(Status::success);\n"
            "\t\t}\n";

        if(skillType == "Action")
        {
            cppCode.handlersCode +=
                "\t\telse if (result == \"RUNNING\" )\n"
                "\t\t{\n"                  
                "\t\t\tm_tickResult.store(Status::running);\n"
                "\t\t}\n";
        }

        cppCode.handlersCode +=
            "\t\telse if (result == \"FAILURE\" )\n"
            "\t\t{ \n"
            "\t\t\tm_tickResult.store(Status::failure);\n"
            "\t\t}\n"
            "\t});\n\n";

    }
    else
    {
        getDataFromEvent(eventData);
        
        std::string nodeName = "node" + eventData.functionName;
        std::string clientName = "client"+ eventData.functionName;

        if(extractInterfaceName(modelFileName, eventData))
        {
            extractInterfaceType(interfaceFileName,eventData);
            printEventData(eventData);
        }
        
        if(eventData.eventType == "send")
        {
            cppCode.handlersCode +=
            "    m_stateMachine.connectToEvent(\"" + eventData.event + "\", [this]([[maybe_unused]]const QScxmlEvent & event){\n"
            "        std::shared_ptr<rclcpp::Node> " + nodeName + " = rclcpp::Node::make_shared(m_name + \"SkillNode" + eventData.functionName + "\");\n";

            if(eventData.interfaceType == "async-service" || eventData.interfaceType == "sync-service" )
            {
                std::string functionNameSnakeCase, serverName;
                turnToSnakeCase(eventData.functionName,functionNameSnakeCase);
                serverName = "\"/"+ eventData.componentName +"/" + eventData.functionName + "\"";
                hCode.includeCode += "#include <" + eventData.interfaceName + "/srv/" + functionNameSnakeCase + ".hpp>\n";
                cppCode.handlersCode +=
                "        std::shared_ptr<rclcpp::Client<"+ eventData.interfaceName +"::srv::" + eventData.functionName + ">> "+ clientName +" = "+ nodeName +"->create_client<"+ eventData.interfaceName +"::srv::" + eventData.functionName + ">(" + serverName +");\n"
                "        auto request = std::make_shared<"+ eventData.interfaceName +"::srv::" + eventData.functionName + "::Request>();\n";
                bool hasParam = false;
                for (auto it =  eventData.paramMap.begin(); it != eventData.paramMap.end(); ++it) {
                    if(!hasParam){
                        cppCode.handlersCode += "        auto eventParams = event.data().toMap();\n";
                        hasParam = true;
                    }    
                    cppCode.handlersCode +=
                        "        request->" + it->first + " = convert<decltype(request->" + it->first + ")>(eventParams[\"" + it->first + "\"].toString().toStdString());\n";
                }
                
                cppCode.handlersCode +=
                "        bool wait_succeded{true};\n"
                "        while (!"+ clientName +"->wait_for_service(std::chrono::seconds(1))) {\n"
                "            if (!rclcpp::ok()) {\n"
                "                RCLCPP_ERROR(rclcpp::get_logger(\"rclcpp\"), \"Interrupted while waiting for the service '" + eventData.functionName + "'. Exiting.\");\n"
                "                wait_succeded = false;\n"
                "                m_stateMachine.submitEvent(\"" + eventData.componentName + "."+ eventData.functionName +".Return\");\n"
                "            } \n"
                "        }\n"
                
                "        if (wait_succeded) {\n"
                "            // send the request                                                                    \n"
                "            auto result = "+ clientName +"->async_send_request(request);\n"
                "            auto futureResult = rclcpp::spin_until_future_complete("+ nodeName +", result);\n"
                "            auto response = result.get();\n"
                "            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) \n"
                "            {\n"
                "                if( response->is_ok ==true) {\n"
                "                    QVariantMap data;\n"
                "                    data.insert(\"result\", \"SUCCESS\");\n";
                if(eventData.interfaceDataField != "")
                {
                    cppCode.handlersCode +=
                    "                    data.insert(\""+ eventData.interfaceDataField +"\", response->" + eventData.interfaceDataField;
                    if(eventData.interfaceDataField == "status")
                    {
                        cppCode.handlersCode += ".status";
                    }
                    cppCode.handlersCode +=
                    ");\n";
                }
                cppCode.handlersCode +=
                "                    m_stateMachine.submitEvent(\"" + eventData.componentName + "."+ eventData.functionName +".Return\", data);\n"
                "                    RCLCPP_INFO(rclcpp::get_logger(\"rclcpp\"), \"" + eventData.componentName + "."+ eventData.functionName +".Return\");\n"
                "                } else {\n"
                "                    QVariantMap data;\n"
                "                    data.insert(\"result\", \"FAILURE\");\n"
                "                    m_stateMachine.submitEvent(\"" + eventData.componentName + "." + eventData.functionName +".Return\", data);\n"
                "                    RCLCPP_INFO(rclcpp::get_logger(\"rclcpp\"), \"" + eventData.componentName + "."+ eventData.functionName +".Return\");\n"
                "                }\n"
                "            }\n"
                "        }\n";
            }

            cppCode.handlersCode +="    });\n\n";
        }
        else if(eventData.eventType == "transition")
        {
            if(eventData.interfaceType =="topic")
            {
                topicCode.publicCode += "\tvoid topic_callback(const " + eventData.interfaceDataType +"::SharedPtr msg);\n";
                topicCode.privateCode += "\trclcpp::Subscription<" + eventData.interfaceDataType +">::SharedPtr m_subscription;\n";
                topicCode.servicesCode += "\tm_subscription = m_node->create_subscription<" + eventData.interfaceDataType +">(\n"
                        "\t\t\"/battery_status\", 10, std::bind(&"+ skillData.className +"::topic_callback, this, std::placeholders::_1));\n\n";
                
                if(eventData.interfaceDataType != "")
                {
                    std::string dataPath;
                    getDataTypePath(eventData.interfaceDataType, dataPath);
                    hCode.includeCode += "#include <" + dataPath +".hpp>\n";
                } 

                topicCode.callbacksCode +=
                    "void "+ skillData.className +"::topic_callback(const " + eventData.interfaceDataType +"::SharedPtr msg) {\n"
                "    QVariantMap data;\n"
                "    data.insert(\""+ eventData.interfaceDataField +"\", msg->" + eventData.interfaceDataField + ");\n"
                "    m_stateMachine.submitEvent(\""+ eventData.event +"\", data);\n"
                "}\n\n";
            }
            // else if(eventData.interfaceType == "async-service" || eventData.interfaceType == "sync-service")
            // {
            // }
        }
        
    }

}

void getEventsCode(const std::vector<tinyxml2::XMLElement*> elementsTransition, const std::vector<tinyxml2::XMLElement*> elementsSend, skillDataStr skillData, hCodeStr& hCode, cppCodeStr& cppCode, topicCodeStr& topicCode)
{
    for (const auto& element : elementsTransition) {
        const char* event = element->Attribute("event");
        const char* target = element->Attribute("target");
    
        if (event && target) 
        {

            std::cout << "\nTransition: event=" << event << ", target=" << target << std::endl;
            eventDataStr eventData;
            eventData.target = target;
            eventData.event = event;
            eventData.eventType = "transition";

            processEvent(eventData, skillData, target, hCode, cppCode, topicCode);
        } 
        else
        {
            std::cerr << "\tMissing attribute in <transition> tag" << std::endl;
        }
    }

    for (const auto& element : elementsSend) {
        const char* event = element->Attribute("event");

        if (event) 
        {
            std::cout << "\nSend: event=" << event << std::endl;
            eventDataStr eventData;
            eventData.event = event;
            eventData.eventType = "send";
            std::vector<tinyxml2::XMLElement*> elementsParam;
            findElementVectorByTag(element, "param", elementsParam);
            for (const auto& element : elementsParam) {
                std::string paramName, paramEvent;
                getElementAttValue(element, "name", paramName);
                getElementAttValue(element, "expr", paramEvent);
                eventData.paramMap[paramName] = paramEvent;
                std::cout << "\tparamName=" << paramName << ", paramExpr=" << eventData.paramMap[paramName] << std::endl;
        
            }

            processEvent(eventData, skillData, "", hCode, cppCode, topicCode);
        } 
        else
        {
            std::cerr << "\tMissing attribute in <send> tag" << std::endl;
        }
    }

}

/* Generate output files */

void writeHCode(const skillDataStr skillData, hCodeStr& code, bool datamodel_mode){
    const std::string className = skillData.className;
    const std::string SMName = skillData.SMName;
    const std::string skillType = skillData.skillType;
    const std::string skillName = skillData.skillName;
    std::string skillTypeLC = skillType;
    for (char &c : skillTypeLC) 
    { 
        c = std::tolower(c); 
    } 
    
    code.includeCode = 
        "/******************************************************************************\n"
        " *                                                                            *\n"
        " * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *\n"
        " * All Rights Reserved.                                                       *\n"
        " *                                                                            *\n"
        " ******************************************************************************/\n\n"
        "# pragma once\n\n"
        "#include <mutex>\n"
        "#include <thread>\n"
        "#include <rclcpp/rclcpp.hpp>\n"
        "#include \"" + className + "SM.h\"\n"
        "#include <bt_interfaces/msg/" + skillTypeLC +"_response.hpp>\n";   
    
    code.statusCode = 
        "enum class Status{\n"
        "\tundefined,\n";
        if(skillType == "Action")
        {
            code.statusCode += "\trunning,\n";
        }
    code.statusCode += 
        "\tsuccess,\n"
        "\tfailure\n"
        "};\n\n";

    code.publicCode += 
        "class " + className + "\n"
        "{\n"
        "public:\n"
        "\t" + className +"(std::string name );\n"
        "\tbool start(int argc, char * argv[]);\n"
        "\tstatic void spin(std::shared_ptr<rclcpp::Node> node);\n";
    code.privateCode = 
        "\nprivate:\n"
        "\tstd::shared_ptr<std::thread> m_threadSpin;\n"
        "\tstd::shared_ptr<rclcpp::Node> m_node;\n"
        "\tstd::mutex m_requestMutex;\n"
        "\tstd::string m_name;\n"
        "\t" + SMName +" m_stateMachine;\n"
        "\t\n";
    code.closeClassCode = 
        "\t\n"
        "};\n\n";

    if(datamodel_mode)
    {
        code.includeCode += "#include \""  + skillName + "SkillDataModel.h\"\n";
        code.privateCode += skillName + "SkillDataModel m_dataModel;\n";
    }
    
}

void writeCppCode(const skillDataStr skillData, cppCodeStr& code, bool datamodel_mode){
    const std::string className = skillData.className;
    const std::string skillType = skillData.skillType;
    code.includeCode = 
        "/******************************************************************************\n"
        " *                                                                            *\n"
        " * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *\n"
        " * All Rights Reserved.                                                       *\n"
        " *                                                                            *\n"
        " ******************************************************************************/\n"
        "\n"
        "#include \"" + className + ".h\"\n"
        "#include <future>\n"
        "#include <QTimer>\n"
        "#include <QDebug>\n"
        "#include <QTime>\n"
        "#include <iostream>\n"
        "#include <QStateMachine>\n\n";

        code.converterCode = "#include <type_traits>\n\n"

            "template<typename T>\n"
            "T convert(const std::string& str) {\n"
            "    if constexpr (std::is_same_v<T, int>) {\n"
            "        return std::stoi(str);\n"
            "    } else if constexpr (std::is_same_v<T, double>) {\n"
            "        return std::stod(str);\n"
            "    } else if constexpr (std::is_same_v<T, float>) {\n"
            "        return std::stof(str);\n"
            "    } \n"
            "    else if constexpr (std::is_same_v<T, std::string>) {\n"
            "        return str;\n"
            "    }\n"
            "    else {\n"
            "        // Handle unsupported types\n"
            "        throw std::invalid_argument(\"Unsupported type conversion\");\n"
            "    }\n"
            "}\n\n";

    code.constructorCode = 
        className + "::"+ className +"(std::string name ) :\n"
        "\t\tm_name(std::move(name))\n"
        "{\n";
     if(datamodel_mode)
     {
        code.constructorCode +=
        "\tm_stateMachine.setDataModel(&m_dataModel);\n";
     }
    code.constructorCode +=
        "}\n\n";
    code.spinCode =
        "void "+ className +"::spin(std::shared_ptr<rclcpp::Node> node)\n"
        "{\n"
        "\trclcpp::spin(node);\n"
        "\trclcpp::shutdown();\n" 
        "}\n\n";
    code.startCode =    
        "bool "+ className +"::start(int argc, char*argv[])\n"
        "{\n"
        "\tif(!rclcpp::ok())\n"
        "\t{\n"
        "\t\trclcpp::init(/*argc*/ argc, /*argv*/ argv);\n"
        "\t}\n\n"
        "\tm_node = rclcpp::Node::make_shared(m_name + \"Skill\");\n"
        "\tRCLCPP_DEBUG_STREAM(m_node->get_logger(), \""+ className +"::start\");\n"
        "\tstd::cout << \""+ className +"::start\";\n\n";

    code.servicesCode= "";

    code.endstartCode =         
        "\tm_stateMachine.start();\n"
        "\tm_threadSpin = std::make_shared<std::thread>(spin, m_node);\n\n"
        "\treturn true;\n"
        "}\n\n";

}

void writeDataModelHCode(const skillDataStr skillData, hCodeStr& code){
    const std::string className = skillData.className;
    
    code.includeCode = 
        "/******************************************************************************\n"
        " *                                                                            *\n"
        " * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *\n"
        " * All Rights Reserved.                                                       *\n"
        " *                                                                            *\n"
        " ******************************************************************************/\n\n"
        "# pragma once\n\n"
        "#include <QScxmlCppDataModel>\n"
        "#include <QVariant>\n"
        "#include <string>\n"
        "#include <thread>\n"
        "#include <rclcpp/rclcpp.hpp>\n";   

    code.publicCode += 
        "class " + className + "DataModel: public QScxmlCppDataModel\n"
        "{\n"
        "    Q_SCXML_DATAMODEL\n\n"
        "public:\n"
        "   " + className +"DataModel() = default;\n"
        "   bool setup(const QVariantMap& initialDataValues) override;\n"
        "   void log(std::string to_log);\n"
        "   //void topic_callback(const " "::SharedPtr msg);\n"
        "   static void spin(std::shared_ptr<rclcpp::Node> node);\n";
    code.privateCode = 
        "\nprivate:\n"
        "   uint m_status;\n"
        "   //rclcpp::Subscription<" ">::SharedPtr m_subscription;\n"
        "   std::shared_ptr<std::thread> m_threadSpin;\n"
        "   std::shared_ptr<rclcpp::Node> m_node;\n";
    code.closeClassCode = 
        "\t\n"
        "};\n\n"
        "Q_DECLARE_METATYPE(::" + className +"SkillDataModel*)";
    
}

void writeDataModelCppCode(const skillDataStr skillData, cppDataModelCodeStr& code){
    const std::string className = skillData.className;
    code.includeCode = 
        "/******************************************************************************\n"
        " *                                                                            *\n"
        " * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *\n"
        " * All Rights Reserved.                                                       *\n"
        " *                                                                            *\n"
        " ******************************************************************************/\n"
        "\n"
        "#include \"" + className + "DataModel.h\"\n"
        "#include <QDebug>\n\n";
    code.spinCode =
        "void "+ className +"DataModel::spin(std::shared_ptr<rclcpp::Node> node)\n"
        "{\n"
        "\trclcpp::spin(node);\n"
        "\trclcpp::shutdown();\n" 
        "}\n\n";
    code.logCode =
        "void "+ className +"DataModel::log(std::string to_log)\n"
        "{\n"
        "\tqInfo(to_log.c_str());\n"
        "}\n\n";
    code.setupCode =    
        "bool "+ className +"DataModel::setup(const QVariantMap& initialDataValues)\n"
        "{\n"
        "\tif(!rclcpp::ok())\n"
        "\t{\n"
        "\t\trclcpp::init(/*argc*/ 0, /*argv*/ nullptr);\n"
        "\t}\n\n"
        "\tm_node = rclcpp::Node::make_shared(\""+ className +"DataModelNode\");\n"
        "\tm_threadSpin = std::make_shared<std::thread>(spin, m_node);\n";

        "\tstd::cout << \""+ className +"::start\";\n\n";

    code.subscriptionCode= "";

    code.endsetupCode =         
        "\treturn true;\n"
        "}\n\n";

    code.setupCode += "\t//m_subscription = m_node->create_subscription<" ">(\n"
    "\t//\t\"/\", 10, std::bind(&""::topic_callback, this, std::placeholders::_1));\n\n";

    code.includeCode += "//#include <"".hpp>\n\n";

    code.callbacksCode +=
        "//void "+ skillData.className +"::topic_callback(const " "::SharedPtr msg) {\n"
    "//}\n\n";

}


void generateDataModelHFile(const std::string outputPath, const std::string outputFileName, hCodeStr code)
{

    
    static std::ofstream outputFile(outputPath + outputFileName);
    if (!outputFile.is_open()) {
        std::cerr << "Failed to open file for writing: " << outputPath + outputFileName << std::endl;
        return;
    }
    outputFile << code.includeCode;
    outputFile << "\n";
    outputFile << code.openClassCode;
    outputFile << code.publicCode;
    outputFile << code.privateCode;
    outputFile << code.closeClassCode;
    outputFile.close();
    std::cout << outputFileName + " file generated" << std::endl;
}

void generateDataModelCppFile(const std::string outputPath, const std::string outputFileName, cppDataModelCodeStr code) 
{
    static std::ofstream outputFile(outputPath + outputFileName);
    if (!outputFile.is_open()) {
        std::cerr << "Failed to open file for writing: " << outputPath + outputFileName << std::endl;
        return;
    }
    
    outputFile << code.includeCode; 
    outputFile << code.spinCode;
    outputFile << code.logCode;
    outputFile << code.setupCode; 
    outputFile << code.subscriptionCode;
    outputFile << code.endsetupCode;
    outputFile << code.callbacksCode; 
    
    outputFile.close();
    std::cout << outputFileName + " file generated" << std::endl;
}

void generateHFile(const std::string outputPath, const std::string outputFileName, const skillDataStr skillData, hCodeStr code) 
{
    const std::string className = skillData.className;
    const std::string skillType = skillData.skillType;
    std::string skillTypeLC = skillType;
    for (char &c : skillTypeLC) 
    { 
        c = std::tolower(c); 
    } 
    
    static std::ofstream outputFile(outputPath + outputFileName);
    if (!outputFile.is_open()) {
        std::cerr << "Failed to open file for writing: " << outputPath + outputFileName << std::endl;
        return;
    }
    if(eventsMap.find("CMD_TICK") != eventsMap.end())
    {
        code.includeCode += "#include <bt_interfaces/srv/tick_" + skillTypeLC +".hpp>\n";
        code.privateCode += 
            "\tstd::atomic<Status> m_tickResult{Status::undefined};\n"
            "\trclcpp::Service<bt_interfaces::srv::Tick" + skillType + ">::SharedPtr m_tickService;\n";
    }
    if(eventsMap.find("CMD_HALT") != eventsMap.end())
    {
        code.includeCode += "#include <bt_interfaces/srv/halt_" + skillTypeLC +".hpp>\n";
        code.privateCode += 
            "\tstd::atomic<bool> m_haltResult{false};\n"
            "\trclcpp::Service<bt_interfaces::srv::Halt" + skillType + ">::SharedPtr m_haltService;\n";
    }

    outputFile << code.includeCode;
    outputFile << "\n"; 
    outputFile << code.statusCode;
    outputFile << code.openClassCode;
    outputFile << code.publicCode;
    outputFile << code.privateCode;
    outputFile << code.closeClassCode;
    outputFile.close();
    std::cout << outputFileName + " file generated" << std::endl;

}

void generateCppFile(const std::string outputPath, const std::string outputFileName, const skillDataStr skillData, cppCodeStr code) 
{
    const std::string className = skillData.className;
    const std::string skillType = skillData.skillType;
    
    if(eventsMap.find("CMD_TICK") != eventsMap.end())
    {
        code.servicesCode +=
        "\tm_tickService = m_node->create_service<bt_interfaces::srv::Tick" + skillType + ">(m_name + \"Skill/tick\",\n"  
        "                                                                           \tstd::bind(&"+ className +"::tick,\n"
        "                                                                           \tthis,\n"
        "                                                                           \tstd::placeholders::_1,\n"
        "                                                                           \tstd::placeholders::_2));\n\n";
        code.callbacksCode +=
            "void " + className + "::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::Tick" + skillType +"::Request> request,\n"
            "                                std::shared_ptr<bt_interfaces::srv::Tick" + skillType +"::Response>      response)\n"
            "{\n"
            "    std::lock_guard<std::mutex> lock(m_requestMutex);\n"
            "    RCLCPP_INFO(m_node->get_logger(), \"" + className + "::tick\");\n"
            "    auto message = bt_interfaces::msg::" + skillType +"Response();\n"
            "    m_tickResult.store(Status::undefined); //here we can put a struct\n"
            "    m_stateMachine.submitEvent(\""+ cmdTick +"\");\n"
            "   \n"
            "    while(m_tickResult.load()== Status::undefined) \n"
            "    {\n"
            "        std::this_thread::sleep_for (std::chrono::milliseconds(100));\n"
            "        // qInfo() <<  \"active names\" << m_stateMachine.activeStateNames();\n"
            "    }\n"
            "    switch(m_tickResult.load()) \n"
            "    {\n";
            if(skillType == "Action")
            {
                code.callbacksCode +=
                "        case Status::running:\n"
                "            response->status.status = message.SKILL_RUNNING;\n"
                "            break;\n";
            }

            code.callbacksCode +=
            "        case Status::failure:\n"
            "            response->status.status = message.SKILL_FAILURE;\n"
            "            break;\n"
            "        case Status::success:\n"
            "            response->status.status = message.SKILL_SUCCESS;\n"
            "            break;            \n"
            "    }\n"
            "    RCLCPP_INFO(m_node->get_logger(), \"" + className + "::tickDone\");\n"
            "   \n"
            "    response->is_ok = true;\n"
            "}\n\n";
    }
    if (eventsMap.find("CMD_HALT") != eventsMap.end())
    {
        code.servicesCode +=
        "\tm_haltService = m_node->create_service<bt_interfaces::srv::Halt" + skillType + ">(m_name + \"Skill/halt\",\n"  
        "                                                                            \tstd::bind(&"+ className +"::halt,\n"
        "                                                                            \tthis,\n"
        "                                                                            \tstd::placeholders::_1,\n"
        "                                                                            \tstd::placeholders::_2));\n\n";   

        code.callbacksCode += 
        "void " + className + "::halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::Halt" + skillType +"::Request> request,\n"
        "    [[maybe_unused]] std::shared_ptr<bt_interfaces::srv::Halt" + skillType +"::Response> response)\n"
        "{\n"
        "    std::lock_guard<std::mutex> lock(m_requestMutex);\n"
        "    RCLCPP_INFO(m_node->get_logger(), \"" + className + "::halt\");\n"
        "    m_haltResult.store(false); //here we can put a struct\n"
        "    m_stateMachine.submitEvent(\"haltCall\");\n"
        "   \n"
        "    while(!m_haltResult.load()) \n"
        "    {\n"
        "        std::this_thread::sleep_for (std::chrono::milliseconds(100));\n"
        "        // qInfo() <<  \"active names\" << m_stateMachine.activeStateNames();\n"
        "    }\n"
        "    RCLCPP_INFO(m_node->get_logger(), \"" + className + "::haltDone\");\n"
        "   \n"
        "    response->is_ok = true;\n"
        "}\n";  
    }    

    static std::ofstream outputFile(outputPath + outputFileName);
    if (!outputFile.is_open()) {
        std::cerr << "Failed to open file for writing: " << outputPath + outputFileName << std::endl;
        return;
    }
    
    outputFile << code.includeCode; 
    outputFile << code.converterCode; 
    outputFile << code.constructorCode; 
    outputFile << code.spinCode; 
    outputFile << code.startCode;
    outputFile << code.servicesCode;
    outputFile << code.handlersCode;
    outputFile << code.endstartCode;
    outputFile << code.callbacksCode; 
    
    outputFile.close();
    std::cout << outputFileName + " file generated" << std::endl;
}

void print_help()
{
    std::cout << "Welcome to SCXMLGenerator tool. Syntax:\n";
    std::cout << "scxmlgen --input_filename \"filename.scxml\" --model_filename \"filename.scxml\" --interface_filename \"filename.scxml\" --output_path \"path/to/output/directory\" [--debug_mode]\n";
}

int main(int argc, char* argv[])
{
    bool debug_mode = false;
    bool datamodel_mode = false;
    std::string input_filename = "in.scxml";
    std::string output_path  = "./";
    

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
        else if (arg == "--output_path" && i+1 < argc && argv[i+1][0] != '-') {
            output_path = argv[i + 1];
            i++;
        }
        else if (arg == "--model_filename" && i+1 < argc && argv[i+1][0] != '-') {
            modelFileName = argv[i + 1];
            i++;
        }
        else if (arg == "--interface_filename" && i+1 < argc && argv[i+1][0] != '-') {
            interfaceFileName = argv[i + 1];
            i++;
        }
        else if (arg == "--datamodel_mode") {
            datamodel_mode = true;
        }
        else if (arg == "--debug_mode") {
            debug_mode = true;
        }
    }
    std::string outputPathHFile = output_path + "/include/";
    std::string outputPathCppFile = output_path + "/src/";
    std::string rootName;
    skillDataStr skillData;
    std::vector<tinyxml2::XMLElement *> elementsTransition, elementsSend;
    tinyxml2::XMLDocument doc;
    std::cout << "-----------" << std::endl;
    if(!extractFromSCXML(doc, input_filename, rootName, elementsTransition, elementsSend)){
        return 0;
    }
    
    if(!getDataFromRootName(rootName, skillData)){
        return 0;
    }
    printSkillData(skillData);
 
    hCodeStr hCode;
    cppCodeStr cppCode;
    topicCodeStr topicCode;
    writeHCode(skillData, hCode, datamodel_mode);
    writeCppCode(skillData, cppCode, datamodel_mode);
    getEventsCode(elementsTransition, elementsSend, skillData, hCode, cppCode, topicCode);
    std::cout << "-----------" << std::endl;
    std::string outputFileNameH = skillData.className + ".h";
    std::string outputFileNameCPP = skillData.className + ".cpp";
    
    if(datamodel_mode)
    {
        hCodeStr hDataModelCode;
        cppDataModelCodeStr cppDataModelCode;
        std::string outputFileNameDataModelH = skillData.className + "DataModel.h";
        std::string outputFileNameDataModelCPP = skillData.className + "DataModel.cpp";   
        writeDataModelHCode(skillData, hDataModelCode);
        writeDataModelCppCode(skillData, cppDataModelCode);
        hDataModelCode.includeCode += topicCode.includeCode;
        hDataModelCode.publicCode += topicCode.publicCode;
        hDataModelCode.privateCode += topicCode.privateCode;
        cppDataModelCode.subscriptionCode += topicCode.servicesCode;
        cppDataModelCode.callbacksCode += topicCode.callbacksCode;
        generateDataModelHFile(outputPathHFile, outputFileNameDataModelH, hDataModelCode);
        generateDataModelCppFile(outputPathCppFile, outputFileNameDataModelCPP, cppDataModelCode);
    }
    else
    {
        hCode.includeCode += topicCode.includeCode;
        hCode.publicCode += topicCode.publicCode;
        hCode.privateCode += topicCode.privateCode;
        cppCode.servicesCode += topicCode.servicesCode;
        cppCode.callbacksCode += topicCode.callbacksCode;
    }

    generateHFile(outputPathHFile, outputFileNameH, skillData, hCode);  
    generateCppFile(outputPathCppFile, outputFileNameCPP, skillData, cppCode);

    
    return 0;
};