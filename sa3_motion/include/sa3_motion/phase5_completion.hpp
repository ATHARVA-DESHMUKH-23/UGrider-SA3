#pragma once
// Phase 5 completion handshake only: no cell-safety validator, no motion gate.
// Sends done/abort to the phase coordinator so Phase 6 begins after both tasks finish.
#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
namespace sa3_phase5 {
using namespace std::chrono_literals;
class CompletionClient {
public:
  CompletionClient(rclcpp::Node::SharedPtr node, std::string owner)
    : node_(std::move(node)), owner_(std::move(owner)) {
    if (owner_ != "robot3" && owner_ != "robot12")
      throw std::runtime_error("Invalid Phase5 completion owner");
  }
  ~CompletionClient() {
    if (!finished_ && rclcpp::ok()) {
      std::string ignored;
      (void)request("/sa3_phase5/" + owner_ + "_abort", ignored);
    }
  }
  bool done() {
    std::string why;
    if (request("/sa3_phase5/" + owner_ + "_done", why)) {
      finished_ = true;
      return true;
    }
    RCLCPP_ERROR(node_->get_logger(), "Phase5 completion not acknowledged for %s: %s",
                 owner_.c_str(), why.c_str());
    return false;
  }
private:
  bool request(const std::string & service, std::string & reason) {
    auto client = node_->create_client<std_srvs::srv::Trigger>(service);
    if (!client->wait_for_service(5s)) {reason = "service unavailable"; return false;}
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(5s) != std::future_status::ready) {
      client->remove_pending_request(future);
      reason = "response timeout";
      return false;
    }
    auto result = future.get();
    reason = result ? result->message : "null response";
    return result && result->success;
  }
  rclcpp::Node::SharedPtr node_;
  std::string owner_;
  bool finished_{false};
};
} // namespace sa3_phase5