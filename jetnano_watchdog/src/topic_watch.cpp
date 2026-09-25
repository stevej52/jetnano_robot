// Copyright 2026 stevej52
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// topic_watch: a live tally of a list of topics, for the watchdog.
//
//   ros2 run jetnano_watchdog topic_watch --ros-args -p topics:="['/scan', '/imu/data']"
//   ros2 topic echo /watchdog/topics
//
// Subscribes to each topic as serialized bytes (GenericSubscription: nothing
// is deserialised, whatever the type) with best-effort QoS, which matches
// reliable and best-effort publishers alike, and once a second publishes a
// DiagnosticArray: one status per topic with its rate over the last
// window_s, the age of the last message and the total count. A topic that
// does not exist yet is looked up again every second until it does.
//
// Why C++: the same eight subscriptions (~160 messages a second) cost a
// Python node 18.6 % of a core in executor overhead, and this node with the
// default executor 4 % (measured 2026-09-25). Hence the events executor, and
// the watchdog leaves the 30 Hz streams (visual odometry, the camera, the
// EKF) to vo_watchdog and to node checks.

#include <chrono>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/experimental/executors/events_executor/events_executor.hpp"
#include "rclcpp/rclcpp.hpp"

using Clock = std::chrono::steady_clock;

namespace
{

struct Watched
{
  std::string type;
  rclcpp::GenericSubscription::SharedPtr sub;
  std::deque<Clock::time_point> stamps;   // arrivals within the window
  Clock::time_point last{};
  bool seen = false;
  uint64_t total = 0;
};

diagnostic_msgs::msg::KeyValue kv(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue k;
  k.key = key;
  k.value = value;
  return k;
}

std::string fixed(double v, int digits)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.*f", digits, v);
  return buf;
}

}  // namespace

class TopicWatch : public rclcpp::Node
{
public:
  TopicWatch()
  : Node("topic_watch")
  {
    auto topics = declare_parameter<std::vector<std::string>>("topics", std::vector<std::string>{});
    window_ = std::chrono::duration<double>(declare_parameter<double>("window_s", 5.0));
    for (const auto & t : topics) {
      watched_[t] = Watched{};
    }
    pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("watchdog/topics", 10);
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {tick();});
    RCLCPP_INFO(get_logger(), "watching %zu topics", watched_.size());
  }

private:
  void subscribe_missing()
  {
    bool missing = false;
    for (auto & [name, w] : watched_) {
      if (!w.sub) {missing = true;}
    }
    if (!missing) {return;}
    const auto graph = get_topic_names_and_types();
    for (auto & [name, w] : watched_) {
      if (w.sub) {continue;}
      auto it = graph.find(name);
      if (it == graph.end() || it->second.empty()) {continue;}
      w.type = it->second.front();
      Watched * wp = &w;
      w.sub = create_generic_subscription(
        name, w.type, rclcpp::SensorDataQoS(),
        [wp](std::shared_ptr<const rclcpp::SerializedMessage>) {
          const auto now = Clock::now();
          wp->stamps.push_back(now);
          wp->last = now;
          wp->seen = true;
          ++wp->total;
        });
      RCLCPP_INFO(get_logger(), "subscribed to %s (%s)", name.c_str(), w.type.c_str());
    }
  }

  void tick()
  {
    subscribe_missing();
    const auto now = Clock::now();
    diagnostic_msgs::msg::DiagnosticArray out;
    out.header.stamp = this->now();
    for (auto & [name, w] : watched_) {
      while (!w.stamps.empty() && now - w.stamps.front() > window_) {
        w.stamps.pop_front();
      }
      double hz = 0.0;
      if (w.stamps.size() >= 2) {
        const double span = std::chrono::duration<double>(w.stamps.back() - w.stamps.front()).count();
        if (span > 0.0) {hz = static_cast<double>(w.stamps.size() - 1) / span;}
      }
      const double age = w.seen ? std::chrono::duration<double>(now - w.last).count() : -1.0;
      diagnostic_msgs::msg::DiagnosticStatus s;
      s.name = name;
      s.hardware_id = w.type;
      s.level = !w.sub ? diagnostic_msgs::msg::DiagnosticStatus::STALE :
        (!w.seen || age > 5.0) ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
        diagnostic_msgs::msg::DiagnosticStatus::OK;
      s.message = !w.sub ? "no such topic" : !w.seen ? "no data yet" : "";
      s.values.push_back(kv("hz", fixed(hz, 2)));
      s.values.push_back(kv("age_s", fixed(age, 2)));
      s.values.push_back(kv("total", std::to_string(w.total)));
      out.status.push_back(std::move(s));
    }
    pub_->publish(out);
  }

  std::map<std::string, Watched> watched_;
  std::chrono::duration<double> window_{5.0};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // The events executor wakes per message instead of rebuilding a wait set:
  // far less overhead at ~100 messages a second.
  rclcpp::experimental::executors::EventsExecutor executor;
  auto node = std::make_shared<TopicWatch>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
