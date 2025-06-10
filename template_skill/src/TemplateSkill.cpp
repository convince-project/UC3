#include "$className$.h"
#include <future>
#include <QTimer>
#include <QDebug>
#include <QTime>
#include <iostream>
#include <QStateMachine>

#include <type_traits>

template<typename T>
T convert(const std::string& str) {
    if constexpr (std::is_same_v<T, int>) {
        return std::stoi(str);
    } else if constexpr (std::is_same_v<T, double>) {
        return std::stod(str);
    } else if constexpr (std::is_same_v<T, float>) {
        return std::stof(str);
    } 
    else if constexpr (std::is_same_v<T, bool>) { 
        if (str == "true" || str == "1") { 
            return true; 
        } else if (str == "false" || str == "0") { 
            return false; 
        } else { 
            throw std::invalid_argument("Invalid boolean value"); 
        } 
    } 
    else if constexpr (std::is_same_v<T, std::string>) {
        return str;
    }
    else {
        throw std::invalid_argument("Unsupported type conversion");
    }
}

$className$::$className$(std::string name ) :
		m_name(std::move(name))
{
    /*DATAMODEL*/m_stateMachine.setDataModel(&m_dataModel);/*END_DATAMODEL*/
}

void $className$::spin(std::shared_ptr<rclcpp::Node> node)
{
	rclcpp::spin(node);
	rclcpp::shutdown();
}

bool $className$::start(int argc, char*argv[])
{
	if(!rclcpp::ok())
	{
		rclcpp::init(/*argc*/ argc, /*argv*/ argv);
	}

	m_node = rclcpp::Node::make_shared(m_name + "Skill");
	RCLCPP_DEBUG_STREAM(m_node->get_logger(), "$className$::start");
	std::cout << "$className$::start";

  /*TICK*/
	m_tickService = m_node->create_service<bt_interfaces_dummy::srv::Tick$skillType$>(m_name + "Skill/tick",
                                                                           	std::bind(&$className$::tick,
                                                                           	this,
                                                                           	std::placeholders::_1,
                                                                           	std::placeholders::_2));/*END_TICK*/
  /*HALT*/
	m_haltService = m_node->create_service<bt_interfaces_dummy::srv::Halt$skillType$>(m_name + "Skill/halt",
                                                                            	std::bind(&$className$::halt,
                                                                            	this,
                                                                            	std::placeholders::_1,
                                                                            	std::placeholders::_2));/*END_HALT*/
  /*ACTION_LIST_C*//*ACTION_C*/
  m_actionClient = rclcpp_action::create_client<$eventData.interfaceName$::action::$eventData.functionName$>(m_node, "/$eventData.componentName$/$eventData.functionName$");
  m_send_goal_options.goal_response_callback = std::bind(&$className$::goal_response_callback, this, std::placeholders::_1);
  m_send_goal_options.feedback_callback =   std::bind(&$className$::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  m_send_goal_options.result_callback =  std::bind(&$className$::result_callback, this, std::placeholders::_1);
  /*END_ACTION_C*/
  /*TOPIC_SUBSCRIPTIONS_LIST*//*TOPIC_SUBSCRIPTION*/
  m_subscription_$eventData.functionName$ = m_node->create_subscription<$eventData.interfaceData[interfaceDataType]$>(
  "/$eventData.functionName$", 10, std::bind(&$className$::topic_callback_$eventData.functionName$, this, std::placeholders::_1));
  /*END_TOPIC_SUBSCRIPTION*/
  /*SEND_EVENT_LIST*//*SEND_EVENT_SRV*/
  m_stateMachine.connectToEvent("$eventData.event$", [this]([[maybe_unused]]const QScxmlEvent & event){
      std::shared_ptr<rclcpp::Node> $eventData.nodeName$ = rclcpp::Node::make_shared(m_name + "SkillNode$eventData.functionName$");
      std::shared_ptr<rclcpp::Client<$eventData.interfaceName$::srv::$eventData.functionName$>> $eventData.clientName$ = $eventData.nodeName$->create_client<$eventData.interfaceName$::srv::$eventData.functionName$>($eventData.serverName$);
      auto request = std::make_shared<$eventData.interfaceName$::srv::$eventData.functionName$::Request>();
      auto eventParams = event.data().toMap();
      /*PARAM_LIST*//*PARAM*/
      request->$IT->FIRST$ = convert<decltype(request->$IT->FIRST$)>(eventParams["$IT->FIRST$"].toString().toStdString());/*END_PARAM*/
      bool wait_succeded{true};
      int retries = 0;
      while (!$eventData.clientName$->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service '$eventData.functionName$'. Exiting.");
              wait_succeded = false;
              break;
          } 
          retries++;
          if(retries == SERVICE_TIMEOUT) {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service '$eventData.functionName$'.");
              wait_succeded = false;
              break;
          }
      }
      if (wait_succeded) {                                                                   
          auto result = $eventData.clientName$->async_send_request(request);
          const std::chrono::seconds timeout_duration(SERVICE_TIMEOUT);
          auto futureResult = rclcpp::spin_until_future_complete($eventData.nodeName$, result, timeout_duration);
          if (futureResult == rclcpp::FutureReturnCode::SUCCESS) 
          {
              auto response = result.get();
              if( response->is_ok == true) {
                  QVariantMap data;
                  data.insert("is_ok", true);/*RETURN_PARAM_LIST*//*RETURN_PARAM*/
                  data.insert("$eventData.interfaceDataField$", response->$eventData.interfaceDataField$/*STATUS*/.status/*END_STATUS*/);/*END_RETURN_PARAM*/
                  m_stateMachine.submitEvent("$eventData.componentName$.$eventData.functionName$.Return", data);
                  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "$eventData.componentName$.$eventData.functionName$.Return");
                  return;
              }
          }
          else if(futureResult == rclcpp::FutureReturnCode::TIMEOUT){
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while future complete for the service '$eventData.functionName$'.");
          }
      }
      QVariantMap data;
      data.insert("is_ok", false);
      m_stateMachine.submitEvent("$eventData.componentName$.$eventData.functionName$.Return", data);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "$eventData.componentName$.$eventData.functionName$.Return");
  });/*END_SEND_EVENT_SRV*/
  /*TICK_RESPONSE*/
  m_stateMachine.connectToEvent("TICK_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
    RCLCPP_INFO(m_node->get_logger(), "$className$::tickReturn %s", event.data().toMap()["status"].toString().toStdString().c_str());
    std::string result = event.data().toMap()["status"].toString().toStdString();
    if (result == std::to_string(SKILL_SUCCESS) )
    {
      m_tickResult.store(Status::success);
    }/*ACTION*/
    else if (result == std::to_string(SKILL_RUNNING) )
    {
      m_tickResult.store(Status::running);
    }/*END_ACTION*/
    else if (result == std::to_string(SKILL_FAILURE) )
    { 
      m_tickResult.store(Status::failure);
    }
  });/*END_TICK_RESPONSE*/
    /*HALT_RESPONSE*/
  m_stateMachine.connectToEvent("HALT_RESPONSE", [this]([[maybe_unused]]const QScxmlEvent & event){
    RCLCPP_INFO(m_node->get_logger(), "$className$::haltresponse");
    m_haltResult.store(true);
  });/*END_HALT_RESPONSE*/

  /*ACTION_LAMBDA_LIST*/
  /*ACTION_SEND_GOAL*/m_stateMachine.connectToEvent("$eventData.componentName$.$eventData.functionName$.SendGoal", [this]([[maybe_unused]]const QScxmlEvent & event){
    RCLCPP_INFO(m_node->get_logger(), "$className$::$eventData.componentName$.$eventData.functionName$.SendGoal");
    RCLCPP_INFO(m_node->get_logger(), "calling send goal");
    std::shared_ptr<rclcpp::Node> node$eventData.functionName$ = rclcpp::Node::make_shared(m_name + "SkillNode$eventData.functionName$");
    rclcpp_action::Client<$eventData.interfaceName$::action::$eventData.functionName$>::SharedPtr client$eventData.functionName$  =
    rclcpp_action::create_client<$eventData.interfaceName$::action::$eventData.functionName$>(node$eventData.functionName$, "/$eventData.componentName$/$eventData.functionName$");
    $eventData.interfaceName$::action::$eventData.functionName$::Goal goal_msg;
    /*SEND_PARAM_LIST*//*SEND_PARAM*/
    std::string temp = event.data().toMap()["$IT->FIRST$"].toString().toStdString();
    goal_msg.$IT->FIRST$ = convert<decltype(goal_msg.$IT->FIRST$)>(temp);
    /*END_SEND_PARAM*/
    send_goal(goal_msg);
    RCLCPP_INFO(m_node->get_logger(), "done send goal");
  });
  /*END_ACTION_SEND_GOAL*/
  /*ACTION_RESULT_REQUEST*/m_stateMachine.connectToEvent("$eventData.componentName$.$eventData.functionName$.ResultRequest", [this]([[maybe_unused]]const QScxmlEvent & event){
      RCLCPP_INFO(m_node->get_logger(), "$className$::$eventData.componentName$.$eventData.functionName$.ResultRequest");
      std::shared_ptr<rclcpp::Node> node$eventData.functionName$ = rclcpp::Node::make_shared(m_name + "SkillNode$eventData.functionName$");
      rclcpp_action::Client<$eventData.interfaceName$::action::$eventData.functionName$>::SharedPtr client$eventData.functionName$  =
        rclcpp_action::create_client<$eventData.interfaceName$::action::$eventData.functionName$>(node$eventData.functionName$, "/$eventData.componentName$/GoToPoi");
      RCLCPP_INFO(m_node->get_logger(), "result request");
  });
  /*END_ACTION_RESULT_REQUEST*/
  /*ACTION_FEEDBACK*/m_stateMachine.connectToEvent("$eventData.componentName$.$eventData.functionName$.Feedback", [this]([[maybe_unused]]const QScxmlEvent & event){
      RCLCPP_INFO(m_node->get_logger(), "$eventData.componentName$.$eventData.functionName$.Feedback");
      QVariantMap data;
      m_feedbackMutex.lock();
      /*FEEDBACK_PARAM_LIST*//*FEEDBACK_PARAM*/
      data.insert("$eventData.interfaceDataField$", m_$eventData.interfaceDataField$);
      /*END_FEEDBACK_PARAM*/
      m_feedbackMutex.unlock();
      m_stateMachine.submitEvent("$eventData.componentName$.$eventData.functionName$.FeedbackReturn", data);
      RCLCPP_INFO(m_node->get_logger(), "$eventData.componentName$.$eventData.functionName$.FeedbackReturn");
  });/*END_ACTION_FEEDBACK*/

	m_stateMachine.start();
	m_threadSpin = std::make_shared<std::thread>(spin, m_node);

	return true;
}
/*TICK_CMD*/
void $className$::tick( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::Tick$skillType$::Request> request,
                                std::shared_ptr<bt_interfaces_dummy::srv::Tick$skillType$::Response>      response)
{
  std::lock_guard<std::mutex> lock(m_requestMutex);
  RCLCPP_INFO(m_node->get_logger(), "$className$::tick");
  m_tickResult.store(Status::undefined);
  m_stateMachine.submitEvent("CMD_TICK");
  
  while(m_tickResult.load()== Status::undefined) {
      std::this_thread::sleep_for (std::chrono::milliseconds(100));
  }
  switch(m_tickResult.load()) 
  {
      /*ACTION*/case Status::running:
          response->status = SKILL_RUNNING;
          break;/*END_ACTION*/
      case Status::failure:
          response->status = SKILL_FAILURE;
          break;
      case Status::success:
          response->status = SKILL_SUCCESS;
          break;  
      case Status::undefined:
          response->status = SKILL_FAILURE;
          break;          
  }
  RCLCPP_INFO(m_node->get_logger(), "$className$::tickDone");
  response->is_ok = true;
}/*END_TICK_CMD*/
/*HALT_CMD*/
void $className$::halt( [[maybe_unused]] const std::shared_ptr<bt_interfaces_dummy::srv::Halt$skillType$::Request> request,
    [[maybe_unused]] std::shared_ptr<bt_interfaces_dummy::srv::Halt$skillType$::Response> response)
{
  std::lock_guard<std::mutex> lock(m_requestMutex);
  RCLCPP_INFO(m_node->get_logger(), "$className$::halt");
  m_haltResult.store(false);
  m_stateMachine.submitEvent("CMD_HALT");
  while(!m_haltResult.load()) {
      std::this_thread::sleep_for (std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(m_node->get_logger(), "$className$::haltDone");
  response->is_ok = true;
}
/*END_HALT_CMD*/


/*TOPIC_CALLBACK_LIST*//*TOPIC_CALLBACK*/
void $className$::topic_callback_$eventData.functionName$(const $eventData.interfaceData[interfaceDataType]$::SharedPtr msg) {
  std::cout << "callback" << std::endl;
  QVariantMap data;
  data.insert("$eventData.interfaceData[interfaceDataField]$", msg->data);

  m_stateMachine.submitEvent("$eventData.componentName$.$eventData.functionName$.Sub", data);
  RCLCPP_INFO(m_node->get_logger(), "$eventData.componentName$.$eventData.functionName$.Sub");
}
/*END_TOPIC_CALLBACK*/

/*ACTION_FNC_LIST*/
/*ACTION_SEND_GOAL_FNC*/
void $className$::send_goal($eventData.interfaceName$::action::$eventData.functionName$::Goal goal_msg)
{
  using namespace std::placeholders;
  bool wait_succeded{true};
  int retries = 0;

  while (!m_actionClient->wait_for_action_server(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service '$eventData.functionName$'. Exiting.");
      wait_succeded = false;
      break;
    } 
    retries++;
    if(retries == SERVICE_TIMEOUT) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Timed out while waiting for the service '$eventData.functionName$'.");
      wait_succeded = false;
      QVariantMap data;
      data.insert("is_ok", false);
      m_stateMachine.submitEvent("$eventData.componentName$.$eventData.functionName$.GoalResponse", data);
      break;
    }
  }
  if (wait_succeded) {
      RCLCPP_INFO(m_node->get_logger(), "Sending goal");
      m_actionClient->async_send_goal(goal_msg, m_send_goal_options);
      QVariantMap data;
      data.insert("is_ok", true);
      m_stateMachine.submitEvent("$eventData.componentName$.$eventData.functionName$.GoalResponse", data);
    }
  }
/*END_ACTION_SEND_GOAL_FNC*/

/*ACTION_RESPONSE_CALLBACK_FNC*/
void $className$::goal_response_callback(const rclcpp_action::ClientGoalHandle<$eventData.interfaceName$::action::$eventData.functionName$>::SharedPtr & goal_handle)
{
  std::cout << "Provaa" << std::endl;
  QVariantMap data;
  if (!goal_handle) {
    data.insert("is_ok", false);
    m_stateMachine.submitEvent("$eventData.componentName$.$eventData.functionName$.GoalResponse", data);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "$eventData.componentName$.$eventData.functionName$.GoalResponse Failure");
    RCLCPP_ERROR(m_node->get_logger(), "Goal was rejected by server");
  } else {
    data.insert("is_ok", true);
    m_stateMachine.submitEvent("$eventData.componentName$.$eventData.functionName$.GoalResponse", data);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "$eventData.componentName$.$eventData.functionName$.GoalResponse Success");
    RCLCPP_INFO(m_node->get_logger(), "Goal accepted by server, waiting for result");
  }
}
/*END_ACTION_RESPONSE_CALLBACK_FNC*/

/*ACTION_FEEDBACK_FNC*/
void $className$::feedback_callback(
    rclcpp_action::ClientGoalHandle<$eventData.interfaceName$::action::$eventData.functionName$>::SharedPtr,
  const std::shared_ptr<const $eventData.interfaceName$::action::$eventData.functionName$::Feedback> feedback)
{
  /*FEEDBACK_PARAM_LIST_FNC*//*FEEDBACK_PARAM_FNC*/
  m_$eventData.interfaceDataField$ = feedback->$eventData.interfaceDataField$;
  /*END_FEEDBACK_PARAM_FNC*/
}
/*END_ACTION_FEEDBACK_FNC*/

/*ACTION_RESULT_CALLBACK_FNC*/
void $className$::result_callback(const  rclcpp_action::ClientGoalHandle<$eventData.interfaceName$::action::$eventData.functionName$>::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(m_node->get_logger(), "Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(m_node->get_logger(), "Goal was canceled");
      break;
    default:
      RCLCPP_ERROR(m_node->get_logger(), "Unknown result code");
      break;
  }
  //std::cout << "Result received: " << result.result->is_ok << std::endl;
  RCLCPP_INFO(m_node->get_logger(), "Result received: %d ", result.result->is_ok);
  QVariantMap data;
  data.insert("is_ok", result.result->is_ok);
  m_stateMachine.submitEvent("$eventData.componentName$.$eventData.functionName$.ResultResponse", data);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "$eventData.componentName$.$eventData.functionName$.ResultResponse");
}/*END_ACTION_RESULT_CALLBACK_FNC*/