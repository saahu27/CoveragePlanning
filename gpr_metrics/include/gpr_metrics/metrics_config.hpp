#ifndef GPR_METRICS__METRICS_CONFIG_HPP_
#define GPR_METRICS__METRICS_CONFIG_HPP_

#include <cstdint>

namespace gpr_metrics
{

/// @brief Tunables for coverage accounting and report export.
struct MetricsConfig
{
  /// @brief First mission id assigned when the baseline is frozen.
  uint64_t mission_id_start{1U};
  /// @brief Child-length deficit below this (m) counts as implicit blocked arc.
  double implicit_blocked_length_epsilon_m{1e-6};
};

}  // namespace gpr_metrics

#endif  // GPR_METRICS__METRICS_CONFIG_HPP_
