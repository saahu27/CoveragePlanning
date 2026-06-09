#ifndef GPR_PLANNING__PLANNING_LOG_HPP_
#define GPR_PLANNING__PLANNING_LOG_HPP_

#include <cstdint>
#include <functional>
#include <string>

namespace gpr_planning
{

enum class PlanningLogLevel : std::uint8_t
{
  Debug = 0,
  Info = 1,
  Warn = 2,
  Error = 3,
};

/// @brief ROS-free logging injected at the composition root (e.g. CoverageStack).
struct PlanningLog
{
  std::function<void(PlanningLogLevel level, const std::string & message)> write;
  std::function<std::int64_t()> now_ns;
};

inline void planning_log_write(
  const PlanningLog & log, const PlanningLogLevel level, const std::string & message)
{
  if (log.write) {
    log.write(level, message);
  }
}

inline void planning_log_debug(const PlanningLog & log, const std::string & message)
{
  planning_log_write(log, PlanningLogLevel::Debug, message);
}

inline void planning_log_info(const PlanningLog & log, const std::string & message)
{
  planning_log_write(log, PlanningLogLevel::Info, message);
}

inline void planning_log_warn(const PlanningLog & log, const std::string & message)
{
  planning_log_write(log, PlanningLogLevel::Warn, message);
}

inline void planning_log_error(const PlanningLog & log, const std::string & message)
{
  planning_log_write(log, PlanningLogLevel::Error, message);
}

inline void planning_log_info_throttle(
  const PlanningLog & log, std::int64_t & last_ns, const std::int64_t period_ms,
  const std::string & message)
{
  if (!log.write || !log.now_ns) {
    return;
  }
  const std::int64_t now = log.now_ns();
  const std::int64_t period = period_ms * 1'000'000LL;
  if (last_ns != 0 && (now - last_ns) < period) {
    return;
  }
  last_ns = now;
  log.write(PlanningLogLevel::Info, message);
}

}  // namespace gpr_planning

#endif  // GPR_PLANNING__PLANNING_LOG_HPP_
