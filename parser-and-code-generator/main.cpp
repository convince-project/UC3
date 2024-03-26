
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

#define modelFileName "./specification/examples/museum-guide/main-XML/full-model.xml"
#define interfaceFileName "./specification/examples/museum-guide/interface-definition-IDL/interfaces.xml"

#define RETURN_CODE_ERROR 1
#define RETURN_CODE_OK    0

static std::map<std::string, bool> eventsMap;

/* Get info from strings*/
void getDataFromEvent(const std::string event, std::string& firstWord, std::string& secondWord, std::string& thirdWord) 
{
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
            std::cerr << "Error in name fomat for event: "<< event << std::endl;
        }
    }
    else{
        std::cerr << "Error in name fomat for event: "<< event << std::endl;
    }
}

void getDataTypePath(const std::string input, std::string& output)
{
    //e.g. from "sensor_msgs::msg::BatteryState" to "sensor_msgs/msg/battery_state"
    std::string temp = input;
    size_t pos = 0;
    bool flag = false;
    if (temp == ""){
        std::cerr << "Input DataType has no value" << std::endl;
        return;
    }

    while ((pos = temp.find("::", pos)) != std::string::npos) {
        temp.replace(pos, 2, "/");
        pos += 1;
    }

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

void getDataFromRootName(const std::string attributeName, std::string& className, std::string& skillName, std::string& skillType)
{
    if (attributeName != ""){
        std::cout << "Root attribute name: " << attributeName << std::endl;
        size_t dotPos = attributeName.find("Skill");
        if (dotPos != std::string::npos){
            skillName = attributeName.substr(0, dotPos);
            className = attributeName.substr(0, dotPos + 5);
            skillType = attributeName.substr(dotPos + 5);
            if(skillType == ""){
                std::cerr << "Skill type not found" << std::endl;
            }
        }
        else{
            std::cerr << "Skill name not found" << std::endl;
        }
    } 
    else{
        std::cerr << "Attribute 'name' not found or has no value" << std::endl;
    }

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


/* Read from XML files */
bool extractInterfaceName(const std::string fileName, const std::string componentName, std::string& interfaceName)
{
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
        std::cerr << "No component '" << componentName << "'found in file '" << fileName << "'."<< std::endl;
        return false;
    }
    if (!getElementAttValue(element, std::string("interface"), interfaceName)){
        std::cerr << "No interface found for component '" << componentName << "'."<< std::endl;
        return false;
    }

    return true;  
}

bool extractInterfaceType(const std::string fileName, const std::string interfaceName, const std::string functionName, std::string& interfaceType, std::string& interfaceDataType)
{
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
    tinyxml2::XMLElement* elementInterface, *elementInterfaceType, *elementFunction, *elementDataType;
    if (!findElementByTagAndAttValue(root, std::string("interface"), std::string("id"), interfaceName, elementInterface))
    {
        std::cerr << "No interface '" << interfaceName << "'found in file '" << fileName << "'."<< std::endl;
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
    if(!findElementByTag(elementFunction, std::string("dataType"), elementDataType))
    {   
        std::cerr << "\tNo tag <dataType> for function '" << functionName << "'."<< std::endl;
        //Not every interface has a defined DataType TODO
        return true;
    } 
    if(!getElementText(elementDataType, interfaceDataType))
    {
         std::cerr << "\tNo value in tag <dataType> for function '" << functionName << "'."<< std::endl;
        //Not every interface has a defined DataType TODO
        return true;
    }
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

void processEvent(const std::string event, const std::string eventType, const std::string className, const std::string skillName, const std::string skillType, std::string& addCode, std::string target, std::string& addIncludeCode, std::string& addPublicCode, std::string& addPrivateCode, std::string& codeCallbacks)
{
    if(eventsMap.find(event) != eventsMap.end())
    {
        // std::cout << "\t" << event << " exists in the map." << std::endl;
        return;
    } 

    // std::cout << "\t" <<  event << " does not exist in the map, adding it." << std::endl;
    eventsMap[event];
    if(event == "CMD_TICK")
    {
        addPublicCode += "\tvoid tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::Tick" + skillType + "::Request> request,\n"
            "\t\t\t   std::shared_ptr<bt_interfaces::srv::Tick" + skillType + "::Response>      response);\n";
    }
    else if(event == "CMD_HALT")
    {
        addPublicCode += "\tvoid halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::Halt" + skillType + "::Request> request,\n"
            "\t\t\t   [[maybe_unused]] std::shared_ptr<bt_interfaces::srv::Halt" + skillType + "::Response> response);\n";
    }
    else if(event == "HALT_RESPONSE")
    {
        addCode += 
            "\tm_stateMachine.connectToEvent(\"haltReturn\", [this]([[maybe_unused]]const QScxmlEvent & event){\n"
            "\t\tRCLCPP_INFO(m_node->get_logger(), \""+ className +"::haltresponse\");\n"
            "\t\tm_haltResult.store(true);\n"
            "\t});\n\n";

    }
    else if(event == "TICK_RESPONSE")
    {
        addCode += 
            "\tm_stateMachine.connectToEvent(\"tickReturn\", [this]([[maybe_unused]]const QScxmlEvent & event){\n"
            "\t\tRCLCPP_INFO(m_node->get_logger(), \"" + className + "::tickReturn %s\", event.data().toMap()[\"result\"].toString().toStdString().c_str());\n"
            "\t\tstd::string result = event.data().toMap()[\"result\"].toString().toStdString();\n"
            "\t\tif (result == \"SUCCESS\" )\n"
            "\t\t{\n" 
            "\t\t\tm_tickResult.store(Status::success);\n"
            "\t\t}\n";

        if(skillType == "Action")
        {
            addCode +=
                "\t\telse if (result == \"RUNNING\" )\n"
                "\t\t{\n"                  
                "\t\t\tm_tickResult.store(Status::running);\n"
                "\t\t}\n";
        }

        addCode +=
            "\t\telse if (result == \"FAILURE\" )\n"
            "\t\t{ \n"
            "\t\t\tm_tickResult.store(Status::failure);\n"
            "\t\t}\n"
            "\t});\n\n";

    }
    else
    {
        std::string componentName = "";
        std::string serviceName = "";
        std::string eventName = "";
        
        getDataFromEvent(event, componentName, serviceName, eventName);
        
        std::string nodeName = "node" + skillName;
        std::string clientName = "client"+ skillName;

        std::cout << "\tcomponent=" << componentName << ", service=" << serviceName << ", eventName=" << eventName << std::endl;
        std::string interfaceName, interfaceType, interfaceDataType;
        extractInterfaceName(modelFileName, componentName, interfaceName);
        extractInterfaceType(interfaceFileName,interfaceName, serviceName, interfaceType, interfaceDataType);

        std::cout<< "\tinterface=" << interfaceName << ", type=" << interfaceType << ", dataType=" << interfaceDataType << std::endl;

        if(eventType == "send")
        {
             addCode +=
            "    m_stateMachine.connectToEvent(\"" + event + "\", [this]([[maybe_unused]]const QScxmlEvent & event){\n"
            "        std::shared_ptr<rclcpp::Node> " + nodeName + " = rclcpp::Node::make_shared(m_name + \"SkillNode" + serviceName + "\");\n";

            if(interfaceType == "async-service" || interfaceType == "sync-service" )
            {
                addIncludeCode += "#include <" + interfaceName + "/srv/" + serviceName + ".hpp>\n\n";
                addCode +=
                "        std::shared_ptr<rclcpp::Client<"+ interfaceName +"::srv::" + serviceName + ">> "+ clientName +" = "+ nodeName +"->create_client<"+ interfaceName +"::srv::" + serviceName + ">(\"/"+ skillName +"Component/" + serviceName + "\");\n"
                
                "        auto request = std::make_shared<"+ interfaceName +"::srv::" + serviceName + "::Request>();\n"
                "        bool wait_succeded{true};\n"
                "        while (!"+ clientName +"->wait_for_service(std::chrono::seconds(1))) {\n"
                "            if (!rclcpp::ok()) {\n"
                "                RCLCPP_ERROR(rclcpp::get_logger(\"rclcpp\"), \"Interrupted while waiting for the service '" + serviceName + "'. Exiting.\");\n"
                "                wait_succeded = false;\n"
                "                m_stateMachine.submitEvent(\"" + componentName + "."+ serviceName +".Return\");\n"
                "            } \n"
                "        }\n"
                
                "        if (wait_succeded) {\n"
                "            // send the request                                                                    \n"
                "            auto result = "+ clientName +"->async_send_request(request);\n"
                "            auto futureResult = rclcpp::spin_until_future_complete("+ nodeName +", result);\n"
                "            if (futureResult == rclcpp::FutureReturnCode::SUCCESS) \n"
                "            {\n"
                "                if( result.get()->is_ok ==true) {\n"
                "                    m_stateMachine.submitEvent(\"" + componentName + "."+ serviceName +".Return\");\n"
                "                    RCLCPP_INFO(rclcpp::get_logger(\"rclcpp\"), \"" + componentName + "."+ serviceName +".Return\");\n"
                "                } else {\n"
                "                    m_stateMachine.submitEvent(\"" + componentName + "." + serviceName +".Return\");\n"
                "                    RCLCPP_INFO(rclcpp::get_logger(\"rclcpp\"), \"" + componentName + "."+ serviceName +".Return\");\n"
                "                }\n"
                "            }\n"
                "        }\n";
            }

            addCode +="    });\n\n";
        }
        else if(eventType == "transition")
        {
            if(interfaceType =="topic")
            {
                addPublicCode += "\tvoid topic_callback(const " + interfaceDataType +"::SharedPtr msg);\n";
                addPrivateCode += "\trclcpp::Subscription<" + interfaceDataType +">::SharedPtr m_subscription;";
                std::string dataPath;
                getDataTypePath(interfaceDataType, dataPath);
                addIncludeCode += "#include <" + dataPath +".hpp>\n\n";

                codeCallbacks +=
                    "void BatteryLevelSkill::topic_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {\n"
                "    QVariantMap data;\n"
                "    data.insert(\"result\", msg->percentage);\n"
                "    m_stateMachine.submitEvent(\""+ event +"\", data);\n"
                "}\n\n";
            }
        }
        
    }

}

void getEventsCode(const std::vector<tinyxml2::XMLElement*> elementsTransition, const std::vector<tinyxml2::XMLElement*> elementsSend, std::string& className, std::string& skillName, std::string& skillType, std::string& includeCode, std::string& publicCode, std::string& privateCode, std::string& handlersCode, std::string& codeCallbacks)
{
    for (const auto& element : elementsTransition) {
        const char* event = element->Attribute("event");
        const char* target = element->Attribute("target");
    
        if (event && target) 
        {
            std::cout << "Transition: event=" << event << ", target=" << target << std::endl;
            processEvent(event, "transition", className, skillName, skillType, handlersCode, target, includeCode, publicCode, privateCode, codeCallbacks);
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
            std::cout << "Send: event=" << event << std::endl;
            processEvent(event, "send", className, skillName, skillType, handlersCode, "", includeCode, publicCode, privateCode, codeCallbacks);
        } 
        else
        {
            std::cerr << "\tMissing attribute in <send> tag" << std::endl;
        }
    }

}

/* Generate output files */

void generateHFile(const std::string className, const std::string skillType, const std::string addIncludeCode, const std::string privateCode, const std::string publicCode) 
{
    std::string skillTypeLC = skillType;
    for (char &c : skillTypeLC) 
    { 
        c = std::tolower(c); 
    } 

    std::string outputFileName = className + ".h";
    std::string includeCode = 
        "# pragma once\n\n"
        "#include <mutex>\n"
        "#include <thread>\n"
        "#include <rclcpp/rclcpp.hpp>\n"
        "#include \"" + className + "SM.h\"\n"
        "#include <bt_interfaces/msg/" + skillTypeLC +"_response.hpp>\n";   

    std::string statusCode = 
        "enum class Status{\n"
        "\tundefined,\n";
        if(skillType == "Action")
        {
            statusCode += "\trunning,\n";
        }
    statusCode += 
        "\tsuccess,\n"
        "\tfailure\n"
        "};\n\n";

    std::string classPublicCode = 
        "class " + className + "\n"
        "{\n"
        "public:\n"
        "\t" + className +"(std::string name );\n"
        "\tbool start(int argc, char * argv[]);\n"
        "\tstatic void spin(std::shared_ptr<rclcpp::Node> node);\n";
    std::string classPrivateCode = 
        "\nprivate:\n"
        "\tstd::shared_ptr<std::thread> m_threadSpin;\n"
        "\tstd::shared_ptr<rclcpp::Node> m_node;\n"
        "\tstd::mutex m_requestMutex;\n"
        "\tstd::string m_name;\n"
        "\t" + className +"SM m_stateMachine;\n"
        "\t\n";
    std::string closeCode = 
        "\t\n"
        "};\n\n";
    
    static std::ofstream outputFile(outputFileName);
    if (!outputFile.is_open()) {
        std::cerr << "Failed to open file for writing: " << outputFileName << std::endl;
        return;
    }
    if(eventsMap.find("CMD_TICK") != eventsMap.end())
    {
        includeCode += "#include <bt_interfaces/srv/tick_" + skillTypeLC +".hpp>\n";
        classPrivateCode += 
            "\tstd::atomic<Status> m_tickResult{Status::undefined};\n"
            "\trclcpp::Service<bt_interfaces::srv::Tick" + skillType + ">::SharedPtr m_tickService;\n";
    }
    if(eventsMap.find("CMD_HALT") != eventsMap.end())
    {
        includeCode += "#include <bt_interfaces/srv/halt_" + skillTypeLC +".hpp>\n";
        classPrivateCode += 
            "\tstd::atomic<bool> m_haltResult{false};\n"
            "\trclcpp::Service<bt_interfaces::srv::Halt" + skillType + ">::SharedPtr m_haltService;\n";
    }

    outputFile << includeCode;
    outputFile << addIncludeCode; 
    outputFile << "\n"; 
    outputFile << statusCode;     
    outputFile << classPublicCode;
    outputFile << publicCode;
    outputFile << classPrivateCode;
    outputFile << privateCode;
    outputFile << closeCode;
    outputFile.close();
    std::cout << outputFileName + " file generated" << std::endl;

}

void generateCppFile(const std::string className, const std::string skillType, const std::string handlersCode, std::string codeCallbacks) 
{
    std::string outputFileName = className + ".cpp";

    std::string includeCode = 
        "\n\n"
        "#include \"" + className + ".h\"\n"
        "#include <QTimer>\n"
        "#include <QDebug>\n"
        "#include <QTime>\n"
        "#include <iostream>\n"
        "#include <QStateMachine>\n\n";

    std::string standardCode = 
        className + "::"+ className +"(std::string name ) :\n"
        "\t\tm_name(std::move(name))\n"
        "{\n"
        "}\n\n"
        "void "+ className +"::spin(std::shared_ptr<rclcpp::Node> node)\n"
        "{\n"
        "\trclcpp::spin(node);\n"
        "\trclcpp::shutdown();\n" 
        "}\n\n"
        "bool "+ className +"::start(int argc, char*argv[])\n"
        "{\n"
        "\tif(!rclcpp::ok())\n"
        "\t{\n"
        "\t\trclcpp::init(/*argc*/ argc, /*argv*/ argv);\n"
        "\t}\n\n"
        "\tm_node = rclcpp::Node::make_shared(m_name + \"Skill\");\n"
        "\tRCLCPP_DEBUG_STREAM(m_node->get_logger(), \""+ className +"::start\");\n"
        "\tstd::cout << \""+ className +"::start\";\n\n";

    std::string startCode = 
        "\tm_stateMachine.start();\n"
        "\tm_threadSpin = std::make_shared<std::thread>(spin, m_node);\n\n";

    std::string closingCode = "}\n\n";

    std::string servicesCode= "";
    if(eventsMap.find("CMD_TICK") != eventsMap.end())
    {
        servicesCode =
        "\tm_tickService = m_node->create_service<bt_interfaces::srv::Tick" + skillType + ">(m_name + \"Skill/tick\",\n"  
        "                                                                           \tstd::bind(&"+ className +"::tick,\n"
        "                                                                           \tthis,\n"
        "                                                                           \tstd::placeholders::_1,\n"
        "                                                                           \tstd::placeholders::_2));\n\n";
        codeCallbacks +=
            "void " + className + "::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces::srv::Tick" + skillType +"::Request> request,\n"
            "                                std::shared_ptr<bt_interfaces::srv::Tick" + skillType +"::Response>      response)\n"
            "{\n"
            "    std::lock_guard<std::mutex> lock(m_requestMutex);\n"
            "    RCLCPP_INFO(m_node->get_logger(), \"" + className + "::tick\");\n"
            "    auto message = bt_interfaces::msg::" + skillType +"Response();\n"
            "    m_tickResult.store(Status::undefined); //here we can put a struct\n"
            "    m_stateMachine.submitEvent(\"tickCall\");\n"
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
                codeCallbacks +=
                "        case Status::running:\n"
                "            response->status.status = message.SKILL_RUNNING;\n"
                "            break;\n";
            }

            codeCallbacks +=
            "        case Status::failure:\n"
            "            response->status.status = message.SKILL_FAILURE;\n"
            "            break;\n"
            "        case Status::success:\n"
            "            response->status.status = message.SKILL_SUCCESS;\n"
            "            break;            \n"
            "    }\n"
            "    RCLCPP_INFO(m_node->get_logger(), \"" + className + "::tickDone\");\n"
            "   \n"
            "response->is_ok = true;\n"
            "}\n\n";
    }
    if (eventsMap.find("CMD_HALT") != eventsMap.end())
    {
        servicesCode +=
        "\tm_haltService = m_node->create_service<bt_interfaces::srv::Halt" + skillType + ">(m_name + \"Skill/halt\",\n"  
        "                                                                            \tstd::bind(&"+ className +"::halt,\n"
        "                                                                            \tthis,\n"
        "                                                                            \tstd::placeholders::_1,\n"
        "                                                                            \tstd::placeholders::_2));\n\n";   

        codeCallbacks += 
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

    static std::ofstream outputFile(outputFileName);
    if (!outputFile.is_open()) {
        std::cerr << "Failed to open file for writing: " << outputFileName << std::endl;
        return;
    }
    
    outputFile << includeCode; 
    outputFile << standardCode; 
    outputFile << servicesCode; 
    outputFile << startCode;
    outputFile << handlersCode;
    outputFile << closingCode;
    outputFile << codeCallbacks; 
    
    outputFile.close();
    std::cout << outputFileName + " file generated" << std::endl;
}

void print_help()
{
    std::cout << "Welcome to SCXMLGenerator tool. Syntax:\n";
    std::cout << "scxmlgen --input_filename \"filename.scxml\" [--output_dir \"output_path\"] [--debug_mode]\n";
   }

int main(int argc, char* argv[])
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
    

    std::string rootName, className, skillType, skillName, includeCode, publicCode, privateCode, handlersCode, codeCallbacks;
    std::vector<tinyxml2::XMLElement *> elementsTransition, elementsSend;
    tinyxml2::XMLDocument doc;
    extractFromSCXML(doc, input_filename, rootName, elementsTransition, elementsSend);
    getDataFromRootName(rootName, className, skillName, skillType);
    std::cout << "-----------" << std::endl;
    std::cout << "Class name: " << className << std::endl << "Skill name: " << skillName << std::endl << "Skill type: " << skillType << std::endl;
    std::cout << "-----------" << std::endl;

    getEventsCode(elementsTransition, elementsSend, className, skillName, skillType, includeCode, publicCode, privateCode, handlersCode, codeCallbacks);
    std::cout << "-----------" << std::endl;
    generateHFile(className, skillType, includeCode, privateCode, publicCode);  
    generateCppFile(className, skillType, handlersCode, codeCallbacks);
    
    return 0;
};