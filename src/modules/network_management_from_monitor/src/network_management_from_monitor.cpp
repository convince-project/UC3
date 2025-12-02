#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "manage_service_interfaces/srv/start_service.hpp"
#include "manage_service_interfaces/srv/stop_service.hpp"

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

    // Subscriber al topic del monitor
    verdict_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/monitor_prop13/monitor_verdict",
      10,
      std::bind(&NetworkManagementFromMonitor::verdictCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "NetworkManagementFromMonitor node started");
  }

private:
  void verdictCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    bool verdict = msg->data;
    RCLCPP_INFO(get_logger(),
                "Received /monitor_prop13/monitor_verdict: %s",
                verdict ? "true" : "false");

    // false -> fai quello che il BT faceva con rete down -> StartService
    // true  -> fai quello che il BT faceva con rete up   -> StopService
    if (!verdict) {
      callStartService();
    } else {
      callStopService();
    }
  }

  void callStartService()
  {
    // Aspetta che il servizio StartService sia disponibile
    int retries = 0;
    while (!start_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(),
                     "Interrupted while waiting for StartService service. Exiting.");
        return;
      }
      ++retries;
      if (retries >= SERVICE_TIMEOUT_SEC) {
        RCLCPP_ERROR(get_logger(),
                     "Timed out while waiting for StartService service.");
        return;
      }
      RCLCPP_WARN(get_logger(), "Waiting for StartService service...");
    }

    auto request = std::make_shared<manage_service_interfaces::srv::StartService::Request>();

    auto future = start_client_->async_send_request(request);

    auto ret = rclcpp::spin_until_future_complete(
      this->shared_from_this(),
      future,
      std::chrono::seconds(SERVICE_TIMEOUT_SEC));

    if (ret == rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      if (response->is_ok) {
        RCLCPP_INFO(get_logger(), "StartService SUCCESS");
      } else {
        RCLCPP_WARN(get_logger(), "StartService returned is_ok=false");
      }
    } else if (ret == rclcpp::FutureReturnCode::TIMEOUT) {
      RCLCPP_ERROR(get_logger(), "StartService call TIMEOUT");
    } else {
      RCLCPP_ERROR(get_logger(), "StartService call FAILED");
    }
  }

  void callStopService()
  {
    // Aspetta che il servizio StopService sia disponibile
    int retries = 0;
    while (!stop_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(),
                     "Interrupted while waiting for StopService service. Exiting.");
        return;
      }
      ++retries;
      if (retries >= SERVICE_TIMEOUT_SEC) {
        RCLCPP_ERROR(get_logger(),
                     "Timed out while waiting for StopService service.");
        return;
      }
      RCLCPP_WARN(get_logger(), "Waiting for StopService service...");
    }

    auto request = std::make_shared<manage_service_interfaces::srv::StopService::Request>();

    auto future = stop_client_->async_send_request(request);

    auto ret = rclcpp::spin_until_future_complete(
      this->shared_from_this(),
      future,
      std::chrono::seconds(SERVICE_TIMEOUT_SEC));

    if (ret == rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      if (response->is_ok) {
        RCLCPP_INFO(get_logger(), "StopService SUCCESS");
      } else {
        RCLCPP_WARN(get_logger(), "StopService returned is_ok=false");
      }
    } else if (ret == rclcpp::FutureReturnCode::TIMEOUT) {
      RCLCPP_ERROR(get_logger(), "StopService call TIMEOUT");
    } else {
      RCLCPP_ERROR(get_logger(), "StopService call FAILED");
    }
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr verdict_sub_;
  rclcpp::Client<manage_service_interfaces::srv::StartService>::SharedPtr start_client_;
  rclcpp::Client<manage_service_interfaces::srv::StopService>::SharedPtr  stop_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NetworkManagementFromMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
