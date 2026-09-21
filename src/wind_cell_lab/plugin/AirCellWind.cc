// Localized wind-force perturbation plus movable, visual-only cell side panels.
// This approximate model is NOT an experimentally validated x500 aerodynamic model.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>
#include "wind_math.hh"

namespace aircells {
using V = gz::math::Vector3d;
using ECM = gz::sim::EntityComponentManager;
using Entity = gz::sim::Entity;
constexpr Entity Nil = gz::sim::kNullEntity;

struct Cell {
  std::string id;
  V center, wind{0, 0, 0}, command{0, 0, 0};
  double yaw{}, length{}, height{}, available{}, width{}, requestedWidth{};
  double commandReceivedSim{-1e9}, widthReceivedSim{-1e9};
  bool windPending{false}, widthPending{false};
  Entity left{Nil}, right{Nil};
};
struct Vehicle {
  std::string name;
  Entity model{Nil}, link{Nil};
  V lastWind{0, 0, 0}, lastForce{0, 0, 0};
  bool limited{false};
};

class AirCellWind : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate,
                    public gz::sim::ISystemPostUpdate {
 public:
  void Configure(const Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 ECM &, gz::sim::EventManager &) override {
    world = entity;
    auto cfg = sdf->Clone();
    rho = cfg->Get<double>("air_density");
    cda = cfg->Get<double>("drag_area");
    cap = cfg->Get<double>("force_cap");
    tau = cfg->Get<double>("wind_response");
    blend = cfg->Get<double>("edge_blend");
    timeout = cfg->Get<double>("command_timeout");
    for (auto e = cfg->GetElement("cell"); e; e = e->GetNextElement("cell")) {
      Cell c;
      c.id = e->Get<std::string>("id");
      c.center = e->Get<V>("center"); c.yaw = e->Get<double>("yaw");
      c.length = e->Get<double>("length"); c.height = e->Get<double>("height");
      c.available = e->Get<double>("available_width");
      c.width = c.requestedWidth = e->Get<double>("initial_width");
      cells.push_back(c);
    }
    for (auto e = cfg->GetElement("vehicle"); e; e = e->GetNextElement("vehicle")) {
      Vehicle v; v.name = e->Get<std::string>(); vehicles.push_back(v);
    }
    for (size_t i = 0; i < cells.size(); ++i) {
      const auto base = "/aircells/" + cells[i].id;
      if (!node.Subscribe<gz::msgs::Vector3d>(base + "/wind_cmd",
          [this, i](const gz::msgs::Vector3d &msg) {
            if (!std::isfinite(msg.x()) || !std::isfinite(msg.y()) || !std::isfinite(msg.z())) return;
            std::lock_guard<std::mutex> guard(lock);
            cells[i].command = V(msg.x(), msg.y(), msg.z());
            cells[i].windPending = true;
          })) throw std::runtime_error("Wind subscription failed");
      if (!node.Subscribe<gz::msgs::Double>(base + "/width_cmd",
          [this, i](const gz::msgs::Double &msg) {
            if (!std::isfinite(msg.data()) || msg.data() <= 0) return;
            std::lock_guard<std::mutex> guard(lock);
            cells[i].requestedWidth = std::min(msg.data(), cells[i].available);
            cells[i].widthPending = true;
          })) throw std::runtime_error("Width subscription failed");
    }
    observations = node.Advertise<gz::msgs::StringMsg>("/aircells/observations");
    gzmsg << "[AirCellWind] ready; " << cells.size()
          << " cells, ideal wind observations, approximate force model\n";
  }

  Entity ModelNamed(const ECM &ecm, const std::string &name) const {
    return ecm.EntityByComponents(gz::sim::components::Model(),
      gz::sim::components::Name(name), gz::sim::components::ParentEntity(world));
  }

  static double Smooth(double margin, double distance) {
    return EdgeWeight(margin, distance);
  }

  V WindAt(const V &p) const {
    V sum(0, 0, 0);
    for (const auto &c : cells) {
      const V d = p - c.center;
      const double along = std::cos(c.yaw) * d.X() + std::sin(c.yaw) * d.Y();
      const double cross = -std::sin(c.yaw) * d.X() + std::cos(c.yaw) * d.Y();
      // The physical weather support is fixed. It never depends on c.width.
      const double weight = Smooth(c.length / 2 - std::abs(along), blend)
                          * Smooth(c.available / 2 - std::abs(cross), blend)
                          * Smooth(c.height / 2 - std::abs(d.Z()), blend);
      sum += c.wind * weight;
    }
    return sum;
  }

  void PreUpdate(const gz::sim::UpdateInfo &info, ECM &ecm) override {
    if (info.paused) return;
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (dt <= 0) return;
    const double now = std::chrono::duration<double>(info.simTime).count();
    std::lock_guard<std::mutex> guard(lock);
    for (auto &c : cells) {
      if (c.windPending) { c.commandReceivedSim = now; c.windPending = false; }
      if (c.widthPending) { c.widthReceivedSim = now; c.widthPending = false; }
      if (now - c.commandReceivedSim > timeout) c.command.Set(0, 0, 0);
      c.wind += (c.command - c.wind) * (1 - std::exp(-dt / tau));
      // A stale controller must not make the cell narrower. Keep its last width.
      c.width = c.requestedWidth;
      if (c.left == Nil) c.left = ModelNamed(ecm, c.id + "_side_left");
      if (c.right == Nil) c.right = ModelNamed(ecm, c.id + "_side_right");
      for (int side : {-1, 1}) {
        const auto id = side < 0 ? c.right : c.left;
        if (id == Nil) continue;
        V p = c.center + V(-std::sin(c.yaw), std::cos(c.yaw), 0) * (side * c.width / 2);
        gz::sim::Model(id).SetWorldPoseCmd(ecm, gz::math::Pose3d(p, gz::math::Quaterniond(0, 0, c.yaw)));
      }
    }
    for (auto &v : vehicles) {
      if (v.model == Nil) v.model = ModelNamed(ecm, v.name);
      if (v.model == Nil) continue;
      if (v.link == Nil) {
        v.link = gz::sim::Model(v.model).CanonicalLink(ecm);
        if (v.link == Nil) continue;
        gz::sim::Link(v.link).EnableVelocityChecks(ecm);
        gzmsg << "[AirCellWind] force attached to " << v.name << "\n";
      }
      gz::sim::Link link(v.link);
      auto pose = link.WorldPose(ecm); auto vel = link.WorldLinearVelocity(ecm);
      if (!pose || !vel) continue;
      v.lastWind = WindAt(pose->Pos());
      // Wind-induced drag increment relative to the existing calm-air model.
      const auto applied = WindIncrement({vel->X(), vel->Y(), vel->Z()},
        {v.lastWind.X(), v.lastWind.Y(), v.lastWind.Z()}, rho, cda, cap);
      V force(applied.force[0], applied.force[1], applied.force[2]);
      v.limited = applied.limited;
      v.lastForce = force;
      link.AddWorldForce(ecm, force);
    }
  }

  static void Vec(std::ostream &out, const V &v) {
    out << '[' << v.X() << ',' << v.Y() << ',' << v.Z() << ']';
  }

  void PostUpdate(const gz::sim::UpdateInfo &info, const ECM &ecm) override {
    if (info.paused) return;
    const double now = std::chrono::duration<double>(info.simTime).count();
    if (now >= lastPublish && now - lastPublish < 0.1) return;
    lastPublish = now;
    std::lock_guard<std::mutex> guard(lock);
    std::ostringstream out; out << std::setprecision(12);
    out << "{\"sim_time\":" << now << ",\"cells\":[";
    bool first = true;
    for (const auto &c : cells) {
      if (!first) out << ',';
      first = false;
      out << "{\"id\":\"" << c.id << "\",\"wind\":"; Vec(out, c.wind);
      out << ",\"width\":" << c.width
          << ",\"panels_found\":" << (c.left != Nil && c.right != Nil ? "true" : "false")
          << ",\"width_command_fresh\":" << (now-c.widthReceivedSim <= timeout ? "true" : "false") << '}';
    }
    out << "],\"vehicles\":["; first = true;
    for (const auto &v : vehicles) {
      if (v.link == Nil) continue;
      gz::sim::Link link(v.link);
      auto pose = link.WorldPose(ecm); auto vel = link.WorldLinearVelocity(ecm);
      if (!pose || !vel) continue;
      if (!first) out << ',';
      first = false;
      out << "{\"name\":\"" << v.name << "\",\"position\":"; Vec(out, pose->Pos());
      out << ",\"velocity\":"; Vec(out, *vel);
      out << ",\"wind\":"; Vec(out, WindAt(pose->Pos()));
      out << ",\"force\":"; Vec(out, v.lastForce);
      out << ",\"force_limited\":" << (v.limited ? "true" : "false") << '}';
    }
    out << "]}";
    gz::msgs::StringMsg msg; msg.set_data(out.str()); observations.Publish(msg);
  }

 private:
  Entity world{Nil};
  std::vector<Cell> cells;
  std::vector<Vehicle> vehicles;
  double rho{}, cda{}, cap{}, tau{}, blend{}, timeout{}, lastPublish{-1};
  std::mutex lock;
  gz::transport::Node node;
  gz::transport::Node::Publisher observations;
};
}
GZ_ADD_PLUGIN(aircells::AirCellWind, gz::sim::System,
              aircells::AirCellWind::ISystemConfigure,
              aircells::AirCellWind::ISystemPreUpdate,
              aircells::AirCellWind::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(aircells::AirCellWind, "aircells::AirCellWind")
