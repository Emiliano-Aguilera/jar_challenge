#pragma once
#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
namespace equipo_jar_navegacion {
struct SharedState {
  std::atomic<bool> locked{false};
};
// Controller and goal checkers live in the same controller_server process/library.
std::shared_ptr<SharedState> shared_state(const std::string& node);
}  // namespace equipo_jar_navegacion
