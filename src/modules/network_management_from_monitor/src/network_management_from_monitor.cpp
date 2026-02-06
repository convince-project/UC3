#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "manage_service_interfaces/srv/start_service.hpp"
#include "manage_service_interfaces/srv/stop_service.hpp"

#include <dialog_interfaces/srv/set_web_status.hpp>

using namespace std::chrono_literals;

static constexpr int SERVICE_TIMEOUT_SEC = 5;

class NetworkManagementFromMonitor : public rclcpp::Node
{
public:
  NetworkManagementFromMonitor()
  : Node("network_management_from_monitor")
  {
    // Client per i servizi Start/Stop reali
    start_client_ = this->create_client<manage_service_interfaces::srv::StartService>(
      "/ManagePeopleDetectorComponent/StartService");

    stop_client_ = this->create_client<manage_service_interfaces::srv::StopService>(
      "/ManagePeopleDetectorComponent/StopService");


    // Client per i servizi Set WebStatus della DialogComponent
    webstatus_client_ = this->create_client<dialog_interfaces::srv::SetWebStatus>(
      "/DialogComponent/SetWebStatus");

    // Client per i servizi Set WebStatus della NarrateComponent
    webstatusNarrate_client_ = this->create_client<dialog_interfaces::srv::SetWebStatus>(
      "/NarrateComponent/SetWebStatus");

    // Subscriber al topic del monitor
    verdict_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/monitor_prop13/monitor_verdict",
      10,
      std::bind(&NetworkManagementFromMonitor::verdictCallback, this, std::placeholders::_1));
    
    verdict_sub_2_ = this->create_subscription<std_msgs::msg::String>(
      "/monitor_prop14/monitor_verdict",
      10,
      std::bind(&NetworkManagementFromMonitor::verdictCallbackDialog, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "NetworkManagementFromMonitor node started");
  }

private:

  void verdictCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string verdict_str = msg->data;
    bool verdict = (verdict_str == "currently_true");

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Received /monitor_prop13/monitor_verdict: %s",
                verdict ? "true" : "false");

    if (m_last_network_status_ && verdict != m_last_network_status_ && verdict==true) {
      callStartService();
    }
    else if (m_last_network_status_ && verdict != m_last_network_status_ && verdict==false) {
      callStopService();
    }

    m_last_network_status_ = verdict;
  }

  void verdictCallbackDialog(const std_msgs::msg::String::SharedPtr msg)
  {

    std::string verdict_str = msg->data;
    bool verdict = (verdict_str == "currently_true");

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Received /monitor_prop14/monitor_verdict: %s",
                verdict ? "true" : "false");

    // Update the Dialog component state
    if (verdict != m_last_web_status_) {
      callSetWebStatusService(verdict);
      m_last_web_status_ = verdict;
    }
  }

  void callSetWebStatusService(bool is_web_reachable)
  {
    // Aspetta che il servizio SetWebStatus sia disponibile
    if (!webstatus_client_->wait_for_service(1s)) {
        RCLCPP_ERROR(get_logger(), "SetWebStatusDialog service not available.");
        return;
    }
    else if (!webstatusNarrate_client_->wait_for_service(1s)) {
        RCLCPP_ERROR(get_logger(), "SetWebStatusNarrate service not available.");
        return;
    }

    auto request = std::make_shared<dialog_interfaces::srv::SetWebStatus::Request>();
    request->is_web_reachable = is_web_reachable;

    auto future = webstatus_client_->async_send_request(
        request,
        [this](rclcpp::Client<dialog_interfaces::srv::SetWebStatus>::SharedFuture response) {
            try {
                if (response.get()->is_ok) {
                    RCLCPP_INFO(get_logger(), "SetWebStatus SUCCESS");
                } else {
                    RCLCPP_WARN(get_logger(), "SetWebStatus returned is_ok=false");
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(get_logger(), "SetWebStatus call failed: %s", e.what());
            }
        });

    auto future_narrate = webstatusNarrate_client_->async_send_request(
        request,
        [this](rclcpp::Client<dialog_interfaces::srv::SetWebStatus>::SharedFuture response) {
            try {
                if (response.get()->is_ok) {
                    RCLCPP_INFO(get_logger(), "SetWebStatusNarrate SUCCESS");
                } else {
                    RCLCPP_WARN(get_logger(), "SetWebStatusNarrate returned is_ok=false");
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(get_logger(), "SetWebStatusNarrate call failed: %s", e.what());
            }
        });
  }

  void callStartService()
  {

    RCLCPP_INFO(get_logger(), "Waiting for StartService...");
    // Aspetta che il servizio StartService sia disponibile
    if (!start_client_->wait_for_service(1s)) {
        RCLCPP_ERROR(get_logger(), "StartService service not available.");
        return;
    }

    auto request = std::make_shared<manage_service_interfaces::srv::StartService::Request>();

    auto future = start_client_->async_send_request(
        request,
        [this](rclcpp::Client<manage_service_interfaces::srv::StartService>::SharedFuture response) {
            try {
                if (response.get()->is_ok) {
                    RCLCPP_INFO(get_logger(), "StartService SUCCESS");
                } else {
                    RCLCPP_WARN(get_logger(), "StartService returned is_ok=false");
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(get_logger(), "StartService call failed: %s", e.what());
            }
        });
  }

  void callStopService()
  {
      RCLCPP_INFO(get_logger(), "Waiting for StopService...");
      // Aspetta che il servizio StopService sia disponibile
      if (!stop_client_->wait_for_service(1s)) {
          RCLCPP_ERROR(get_logger(), "StopService service not available.");
          return;
      }

      auto request = std::make_shared<manage_service_interfaces::srv::StopService::Request>();

      auto future = stop_client_->async_send_request(
          request,
          [this](rclcpp::Client<manage_service_interfaces::srv::StopService>::SharedFuture response) {
              try {
                  if (response.get()->is_ok) {
                      RCLCPP_INFO(get_logger(), "StopService SUCCESS");
                  } else {
                      RCLCPP_WARN(get_logger(), "StopService returned is_ok=false");
                  }
              } catch (const std::exception &e) {
                  RCLCPP_ERROR(get_logger(), "StopService call failed: %s", e.what());
              }
          });
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr verdict_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr verdict_sub_2_;
  rclcpp::Client<manage_service_interfaces::srv::StartService>::SharedPtr start_client_;
  rclcpp::Client<manage_service_interfaces::srv::StopService>::SharedPtr  stop_client_;
  rclcpp::Client<dialog_interfaces::srv::SetWebStatus>::SharedPtr webstatus_client_;
  rclcpp::Client<dialog_interfaces::srv::SetWebStatus>::SharedPtr webstatusNarrate_client_;
  // assume starting with web and network connections on
  bool m_last_web_status_{true};
  bool m_last_network_status_{true};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NetworkManagementFromMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
