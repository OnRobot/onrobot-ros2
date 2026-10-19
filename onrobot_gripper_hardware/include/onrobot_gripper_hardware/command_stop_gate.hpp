#pragma once

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <utility>
#include <vector>

#include <hardware_interface/handle.hpp>

namespace onrobot_gripper_hardware {

/// @cond INTERNAL
// Keep the same Stop handle (and mutex) in the resource manager and backend.
// Only Stop may be written from the action executor; the manager update thread
// writes motion commands. Hold this lock while trying to queue a command so a
// callback's Stop cannot race with a backend write. Neither side waits for
// device communication. If the lock is busy, the backend retries next cycle.
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
/// @endcond

} // namespace onrobot_gripper_hardware
