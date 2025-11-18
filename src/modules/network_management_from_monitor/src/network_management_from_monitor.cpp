#include <chrono>
#include <memory>
#include <string>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

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

    // read stable duration from param (seconds)
    double stable_secs = this->declare_parameter<double>("verdict_stable_seconds", 2.0);
    stable_duration_ = std::chrono::duration<double>(stable_secs);

    // Client per i servizi Start/Stop reali
    start_client_ = this->create_client<manage_service_interfaces::srv::StartService>(
      "/ManagePeopleDetectorComponent/StartService");

    stop_client_ = this->create_client<manage_service_interfaces::srv::StopService>(
      "/ManagePeopleDetectorComponent/StopService");

    // Subscriber al topic del monitor
    verdict_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/monitor_prop13/monitor_verdict",
      10,
      std::bind(&NetworkManagementFromMonitor::verdictCallback, this, std::placeholders::_1));

    // Timer to check stability of candidate verdict and commit after stable_duration_
    stability_timer_ = this->create_wall_timer(200ms, std::bind(&NetworkManagementFromMonitor::stabilityTimerCb, this));
    RCLCPP_INFO(get_logger(), "NetworkManagementFromMonitor node started");
  }

private:
  void verdictCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Received verdict callback: '%s'", msg->data.c_str());

    bool msg_verdict = false;
    if (msg->data == "currently_true" || msg->data == "true") {
      msg_verdict = true;
    } else if (msg->data == "currently_false" || msg->data == "false") {
      msg_verdict = false;
    } else {
      RCLCPP_WARN(get_logger(), "Unknown verdict string '%s', treating as false", msg->data.c_str());
      msg_verdict = false;
    }

    // If candidate changed, reset candidate_since_
    if (!candidate_verdict_.has_value() || candidate_verdict_.value() != msg_verdict) {
      candidate_verdict_ = msg_verdict;
      candidate_since_ = std::chrono::steady_clock::now();
      RCLCPP_DEBUG(get_logger(), "New candidate verdict=%s at reset time",
                   msg_verdict ? "true" : "false");
    } else {
      // candidate unchanged, nothing to do here; timer will check elapsed time
      RCLCPP_DEBUG(get_logger(), "Candidate verdict unchanged (%s)", msg_verdict ? "true" : "false");
    }
  }

  void stabilityTimerCb()
  {
    if (!candidate_verdict_.has_value()) return;
    auto now = std::chrono::steady_clock::now();
    auto elapsed = now - candidate_since_;
    if (elapsed >= stable_duration_) {
      bool candidate = candidate_verdict_.value();
      if (!prev_verdict_.has_value() || prev_verdict_.value() != candidate) {
        // commit change
        prev_verdict_ = candidate;
        RCLCPP_INFO(get_logger(), "Verdict stable for %.3fs -> committing: %s",
                    std::chrono::duration<double>(elapsed).count(),
                    candidate ? "true" : "false");
        if (!candidate) {
          callStartService();
        } else {
          callStopService();
        }
      } else {
        RCLCPP_DEBUG(get_logger(), "Verdict stable but equal to previous applied verdict; no action");
      }
      // keep candidate as is (it matches prev now) but avoid repeated invokes:
      // clear candidate to wait for future changes
      candidate_verdict_.reset();
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

    if (future.wait_for(std::chrono::seconds(SERVICE_TIMEOUT_SEC)) == std::future_status::ready) {
      auto response = future.get();
      if (response->is_ok) {
        RCLCPP_INFO(get_logger(), "StartService SUCCESS");
      } else {
        RCLCPP_WARN(get_logger(), "StartService returned is_ok=false");
      }
    } else {
      RCLCPP_ERROR(get_logger(), "StartService call TIMEOUT or FAILED");
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

    if (future.wait_for(std::chrono::seconds(SERVICE_TIMEOUT_SEC)) == std::future_status::ready) {
      auto response = future.get();
      if (response->is_ok) {
        RCLCPP_INFO(get_logger(), "StopService SUCCESS");
      } else {
        RCLCPP_WARN(get_logger(), "StopService returned is_ok=false");
      }
    } else {
      RCLCPP_ERROR(get_logger(), "StopService call TIMEOUT or FAILED");
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr verdict_sub_;
  rclcpp::Client<manage_service_interfaces::srv::StartService>::SharedPtr start_client_;
  rclcpp::Client<manage_service_interfaces::srv::StopService>::SharedPtr  stop_client_;
  std::optional<bool> prev_verdict_;

  // stability logic
  std::optional<bool> candidate_verdict_;
  std::chrono::steady_clock::time_point candidate_since_;
  std::chrono::duration<double> stable_duration_;
  rclcpp::TimerBase::SharedPtr stability_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NetworkManagementFromMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
