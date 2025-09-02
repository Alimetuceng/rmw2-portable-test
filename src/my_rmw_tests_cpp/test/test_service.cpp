#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <chrono>
#include <future>

using namespace std::chrono_literals;

TEST(Service, AddTwoIntsRoundTrip) {
  int argc = 0; rclcpp::init(argc, nullptr);
  auto node = std::make_shared<rclcpp::Node>("svc_node");
  auto service = node->create_service<example_interfaces::srv::AddTwoInts>(
    "add",
    [](const example_interfaces::srv::AddTwoInts::Request::SharedPtr req,
       example_interfaces::srv::AddTwoInts::Response::SharedPtr res) {
      res->sum = req->a + req->b;
    });

  auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add");
  ASSERT_TRUE(client->wait_for_service(2s));

  auto req = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  req->a = 2; req->b = 3;

  auto fut = client->async_send_request(req);
  auto start = std::chrono::steady_clock::now();
  while (fut.wait_for(10ms) != std::future_status::ready) {
    rclcpp::spin_some(node);
    if (std::chrono::steady_clock::now() - start > 2s) break;
  }

  ASSERT_EQ(fut.wait_for(0s), std::future_status::ready) << "Service timeout";
  EXPECT_EQ(fut.get()->sum, 5);
  rclcpp::shutdown();
}
