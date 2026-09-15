// Integration fixture: publish exact synthetic world-pose scans over Gazebo.
#include <gz/msgs/laserscan.pb.h>
#include <gz/transport/Node.hh>
#include <chrono>
#include <cmath>
#include <thread>

int main() {
  gz::transport::Node node;
  auto a = node.Advertise<gz::msgs::LaserScan>("/test/robot1/scan");
  auto b = node.Advertise<gz::msgs::LaserScan>("/test/robot2/scan");
  for (int i=0; i<100 && (!a.HasConnections() || !b.HasConnections()); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  if (!a.HasConnections() || !b.HasConnections()) return 2;
  gz::msgs::LaserScan msg;
  msg.set_count(1); msg.set_vertical_count(1);
  msg.set_range_min(0); msg.set_range_max(10);
  msg.mutable_world_pose()->mutable_position()->set_x(.5);
  msg.mutable_world_pose()->mutable_position()->set_y(.5);
  msg.mutable_world_pose()->mutable_orientation()->set_w(1);
  msg.add_ranges(3);
  auto send = [&](gz::transport::Node::Publisher &pub, int stamp) {
    msg.mutable_header()->mutable_stamp()->set_sec(stamp);
    pub.Publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  };
  send(a, 1); send(a, 2); send(b, 1);
  msg.mutable_world_pose()->mutable_position()->set_x(3.5);
  msg.set_ranges(0, 2);
  send(b, 2);
  msg.mutable_world_pose()->mutable_position()->set_x(5.5);
  msg.mutable_world_pose()->mutable_orientation()->set_w(std::sqrt(.5));
  msg.mutable_world_pose()->mutable_orientation()->set_z(std::sqrt(.5));
  send(a, 3);
}
