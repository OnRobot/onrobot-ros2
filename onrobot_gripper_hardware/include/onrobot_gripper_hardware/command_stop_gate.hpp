#pragma once

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <utility>
#include <vector>

#include <hardware_interface/handle.hpp>

namespace onrobot_gripper_hardware {

// Keep the same Stop handle (and mutex) in the resource manager and backend.
// Only Stop may be written from the action executor; motion outputs belong to
// the manager update thread. Holding this gate through nonblocking admission
// orders a callback's Stop against an in-progress write. Neither side waits on
// transport I/O. A contended backend retries on its next write cycle.
class CommandStopGate {
public:
  std::vector<hardware_interface::CommandInterface::SharedPtr> export_interfaces(
      std::vector<hardware_interface::CommandInterface> interfaces) {
    std::vector<hardware_interface::CommandInterface::SharedPtr> shared;
    shared.reserve(interfaces.size());
    stop_.reset();
    for (auto &interface : interfaces) {
      auto handle = std::make_shared<hardware_interface::CommandInterface>(
          std::move(interface));
      if (handle->get_interface_name() == "stop_command_sequence") {
        stop_ = handle;
      }
      shared.push_back(std::move(handle));
    }
    return shared;
  }

  std::unique_lock<std::shared_mutex> try_lock() {
    return std::unique_lock<std::shared_mutex>(
        stop_ ? stop_->get_mutex() : unused_, std::try_to_lock);
  }

private:
  hardware_interface::CommandInterface::SharedPtr stop_;
  std::shared_mutex unused_;
};

} // namespace onrobot_gripper_hardware
