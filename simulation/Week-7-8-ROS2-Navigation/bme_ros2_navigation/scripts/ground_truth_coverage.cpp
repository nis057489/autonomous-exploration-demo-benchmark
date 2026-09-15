// Gazebo's native LaserScan carries world_pose at the sensor timestamp. The
// ROS LaserScan bridge discards that field; neither ROS TF nor odom is truth.
#include "observed_cells.hpp"
#include <gz/msgs/laserscan.pb.h>
#include <gz/transport/Node.hh>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>

namespace {
volatile std::sig_atomic_t stopping = 0;
void stop(int) { stopping = 1; }
}

int main(int argc, char **argv) {
  if (argc < 4) {
    std::cerr << "Usage: ground_truth_coverage output.csv resolution robot=/scan [...]\n";
    return 2;
  }
  try {
    ObservedCells cells(std::stod(argv[2]), argc - 3);
    std::ofstream out(argv[1]);
    if (!out) throw std::runtime_error("cannot create coverage CSV");
    out << "wall_ns,sim_time_s,robot,scans,observed_cells,union_cells,overlap_cells,resolution_m\n";
    out << std::setprecision(17);
    std::mutex mutex;
    std::vector<std::uint64_t> scans(argc - 3, 0);
    std::vector<double> last_stamp(argc - 3, -1);
    std::atomic<bool> failed{false};
    std::vector<std::string> topics;
    gz::transport::Node node;
    for (unsigned robot = 0; robot < scans.size(); ++robot) {
      const std::string arg = argv[robot + 3];
      const auto split = arg.find('=');
      if (split == std::string::npos) throw std::runtime_error("expected robot=topic");
      const auto name = arg.substr(0, split), topic = arg.substr(split + 1);
      topics.push_back(topic);
      std::function<void(const gz::msgs::LaserScan &)> callback =
          [&, robot, name](const gz::msgs::LaserScan &msg) {
        std::lock_guard<std::mutex> lock(mutex);
        const auto wall = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        const double stamp = msg.header().stamp().sec() + msg.header().stamp().nsec() * 1e-9;
        const auto &p = msg.world_pose().position();
        const auto &q = msg.world_pose().orientation();
        const double norm = std::sqrt(q.w()*q.w()+q.x()*q.x()+q.y()*q.y()+q.z()*q.z());
        if (!msg.has_world_pose() || !msg.has_header() || norm < 0.5 ||
            !std::isfinite(norm) || !std::isfinite(p.x()) || !std::isfinite(p.y()) ||
            msg.vertical_count() != 1 || std::abs(msg.vertical_angle_min()) > 1e-6 ||
            msg.count() == 0 || msg.ranges_size() != int(msg.count()) ||
            !std::isfinite(msg.angle_min()) || !std::isfinite(msg.angle_step()) ||
            !std::isfinite(msg.range_min()) || !std::isfinite(msg.range_max()) ||
            msg.range_min() < 0 || msg.range_max() <= msg.range_min()) {
          if (!failed.exchange(true)) std::cerr << "Invalid ground-truth scan for " << name << '\n';
          return;
        }
        if (stamp < last_stamp[robot]) {
          if (!failed.exchange(true)) std::cerr << "Simulation time reset; start a new recording\n";
          return;
        }
        if (stamp == last_stamp[robot]) return;
        last_stamp[robot] = stamp;
        const double w=q.w()/norm, x=q.x()/norm, y=q.y()/norm, z=q.z()/norm;
        for (int i=0; i<msg.ranges_size(); ++i) {
          const double range = msg.ranges(i);
          if (std::isnan(range) || range < msg.range_min()) continue;
          const double length = std::min(range, msg.range_max());
          const double angle = msg.angle_min() + i * msg.angle_step();
          const double c=std::cos(angle), s=std::sin(angle);
          const double dx=(1-2*(y*y+z*z))*c + 2*(x*y-z*w)*s;
          const double dy=2*(x*y+z*w)*c + (1-2*(x*x+z*z))*s;
          cells.ray(robot, p.x()+dx*msg.range_min(), p.y()+dy*msg.range_min(),
                    p.x()+dx*length, p.y()+dy*length);
        }
        ++scans[robot];
        out << wall << ',' << stamp << ',' << name << ',' << scans[robot] << ','
            << cells.own[robot] << ',' << cells.union_count() << ',' << cells.overlap
            << ',' << cells.resolution_ << '\n';
        out.flush();
        if (!out) failed = true;
      };
      if (!node.Subscribe(topic, callback)) throw std::runtime_error("cannot subscribe " + topic);
      std::cerr << "Ground-truth coverage: " << name << " from " << topic << '\n';
    }
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    while (!stopping && !failed) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    for (const auto &topic : topics) node.Unsubscribe(topic);
    std::lock_guard<std::mutex> lock(mutex);
    for (unsigned i=0; i<scans.size(); ++i)
      if (!scans[i]) { std::cerr << "No ground-truth scans for robot index " << i << '\n'; failed=true; }
    if (failed) {
      // A partial evaluation must never masquerade as complete team coverage.
      std::ofstream(std::string(argv[1]) + ".invalid") << "See ground_truth_coverage.log\n";
    } else {
      std::ofstream(std::string(argv[1]) + ".complete") << "All configured robots observed\n";
    }
    return failed ? 1 : 0;
  } catch (const std::exception &e) {
    std::ofstream(std::string(argv[1]) + ".invalid") << e.what() << '\n';
    std::cerr << e.what() << '\n';
    return 1;
  }
}
