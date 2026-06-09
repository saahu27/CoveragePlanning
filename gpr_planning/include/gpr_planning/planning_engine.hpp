#ifndef GPR_PLANNING__PLANNING_ENGINE_HPP_
#define GPR_PLANNING__PLANNING_ENGINE_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "gpr_common/coverage_types.hpp"
#include "gpr_common/swath_coverage.hpp"
#include "gpr_common/grid_map.hpp"
#include "gpr_common/types.hpp"
#include "gpr_metrics/coverage_reporter.hpp"
#include "gpr_metrics/metrics_config.hpp"
#include "gpr_planning/astar_grid_planner.hpp"
#include "gpr_planning/boustrophedon_planner.hpp"
#include "gpr_planning/boustrophedon_sequencer.hpp"
#include "gpr_planning/i_sequence_solver.hpp"
#include "gpr_planning/oarp_lite_planner.hpp"
#include "gpr_planning/path_invalidator.hpp"
#include "gpr_planning/planning_log.hpp"
#include "gpr_planning/segment_catalog.hpp"

namespace gpr_planning
{

struct CoveragePass;

/// @brief Logging and diagnostic thresholds inside PlanningEngine.
struct PlanningDiagnosticsConfig
{
  double update_grid_log_throttle_ms{2000.0};
  /// @brief Warn when this fraction of segments are blocked on first grid sync.
  double first_grid_blocked_warn_fraction{0.5};
};

/// @brief Domain planning: catalog, schedule, solvers (no ROS publishers).
class PlanningEngine
{
public:
  PlanningEngine(
    BoustrophedonConfig boustrophedon_config,
    PathInvalidatorConfig invalidator_config,
    AStarConfig astar_config,
    std::string frame_id,
    AtspSolverConfig atsp_config = {},
    SegmentCatalogConfig catalog_config = {},
    OarpLiteConfig oarp_config = {},
    bool use_boustrophedon_sequencer = true,
    BoustrophedonSequencerConfig boustrophedon_sequencer_config = {},
    bool show_completed_in_markers = false,
    bool require_reachable_transit = true,
    gpr_common::SwathCoverageConfig swath_coverage_config = {},
    double transit_skip_distance = 0.35,
    double executed_trace_step_m = 0.05,
    double flush_pose_min_step_m = 1e-4,
    PlanningDiagnosticsConfig diagnostics_config = {},
    gpr_metrics::MetricsConfig metrics_config = {});

  void set_log(PlanningLog log) noexcept;
  void set_viz_refresh_callback(std::function<void()> callback);

  void generate_initial_coverage();
  void set_route_end_pose(std::optional<gpr_common::Pose2D> pose) noexcept
  {
    route_end_pose_ = std::move(pose);
  }
  void update_from_grid(gpr_common::GridMapConstPtr grid);
  void apply_snapshot_grid(gpr_common::GridMapConstPtr grid, std::uint64_t grid_seq);
  [[nodiscard]] bool recompute_schedule(const gpr_common::Pose2D & robot_pose);
  [[nodiscard]] std::optional<gpr_common::CoverageJob> pop_next_job(
    const std::optional<gpr_common::Pose2D> & robot_pose = std::nullopt,
    double transit_skip_distance = 0.35);
  [[nodiscard]] gpr_common::Polyline current_job_polyline() const;
  [[nodiscard]] gpr_common::Polyline current_job_polyline_for_robot(
    const gpr_common::Pose2D & robot, double prune_distance_m = 0.15) const;
  [[nodiscard]] gpr_common::Polyline current_job_followable_polyline_for_robot(
    const gpr_common::Pose2D & robot, double prune_distance_m = 0.15) const;
  [[nodiscard]] bool is_current_job_followable(
    const gpr_common::Pose2D & robot, double prune_distance_m = 0.15) const;
  [[nodiscard]] std::optional<gpr_common::Polyline> transit_to_current_job(
    const gpr_common::Pose2D & robot_pose) const;
  [[nodiscard]] std::optional<gpr_common::Polyline> plan_transit_path(
    const gpr_common::Pose2D & from, const gpr_common::Pose2D & to) const;

  void mark_current_job_attempted();
  void set_last_executed_path(const gpr_common::Polyline & path);
  /// @brief Apply swath coverage from the current trace without closing the job.
  void record_current_job_coverage();
  void mark_current_job_complete();
  void mark_current_job_complete(const gpr_common::Polyline & driven_path);
  [[nodiscard]] bool append_executed_pose(
    const gpr_common::Pose2D & pose,
    gpr_common::ExecutionTracePhase phase = gpr_common::ExecutionTracePhase::Coverage);
  /// @brief Append @p pose to the coverage trace without the 5 cm decimation gate.
  void flush_coverage_pose(const gpr_common::Pose2D & pose);
  void clear_executed_trace();
  [[nodiscard]] const gpr_common::Polyline & executed_coverage_trace() const noexcept
  {
    return executed_coverage_trace_;
  }
  [[nodiscard]] const gpr_common::Polyline & mission_executed_trace() const noexcept;
  [[nodiscard]] bool job_completion_satisfied() const;
  void skip_current_job();
  void mark_current_job_blocked();
  [[nodiscard]] std::optional<double> current_job_entry_distance(
    const gpr_common::Pose2D & robot) const;

  [[nodiscard]] const SegmentCatalog & catalog() const noexcept {return catalog_;}
  [[nodiscard]] std::uint64_t schedule_revision() const noexcept {return schedule_revision_;}
  [[nodiscard]] std::uint64_t grid_seq() const noexcept {return grid_seq_;}
  [[nodiscard]] bool has_pending_jobs() const noexcept;
  [[nodiscard]] bool has_work_remaining() const noexcept;
  [[nodiscard]] bool reachability_exhausted() const noexcept;
  [[nodiscard]] bool catalog_changed() const noexcept {return catalog_changed_;}
  void clear_catalog_changed_flag() noexcept {catalog_changed_ = false;}
  [[nodiscard]] bool schedule_needs_rebuild() const noexcept;
  [[nodiscard]] const gpr_common::Polyline & initial_path() const noexcept
  {
    return initial_path_;
  }
  [[nodiscard]] const gpr_metrics::CoverageReporter & reporter() const noexcept
  {
    return reporter_;
  }
  [[nodiscard]] bool has_active_job() const noexcept {return current_job_.has_value();}
  [[nodiscard]] bool active_job_still_actionable() const;
  void release_current_job();
  [[nodiscard]] bool active_job_needs_replan(
    const gpr_common::Pose2D & robot,
    double prune_distance_m,
    double transit_skip_distance,
    bool in_transit_phase) const;
  [[nodiscard]] const gpr_common::Polyline & active_transit_path() const noexcept
  {
    return active_transit_path_;
  }
  void set_active_transit_path(const gpr_common::Polyline & path);
  void clear_active_transit_path();
  [[nodiscard]] std::optional<gpr_common::Pose2D> current_job_entry_pose() const;
  [[nodiscard]] std::optional<gpr_common::Pose2D> job_entry_pose(
    const gpr_common::CoverageJob & job) const;
  [[nodiscard]] bool job_is_followable(
    const gpr_common::CoverageJob & job,
    const gpr_common::Pose2D & robot,
    double prune_distance_m = 0.15) const;
  void refresh_transit_display(
    const gpr_common::Pose2D & robot_pose, double transit_skip_distance);
  [[nodiscard]] bool is_transit_path_followable(const gpr_common::Polyline & path) const;

  [[nodiscard]] const std::vector<gpr_common::CoverageJob> & schedule() const noexcept
  {
    return schedule_;
  }
  [[nodiscard]] const std::string & frame_id() const noexcept {return frame_id_;}
  [[nodiscard]] const gpr_common::GridMap & latest_grid() const noexcept;
  [[nodiscard]] const PathInvalidatorConfig & invalidator_config() const noexcept
  {
    return invalidator_config_;
  }
  [[nodiscard]] bool show_completed_in_markers() const noexcept
  {
    return show_completed_in_markers_;
  }

private:
  friend class RosIoFacade;

  void notify_viz_refresh();
  [[nodiscard]] const CoveragePass * find_pass(gpr_common::SegmentId job_id) const;
  [[nodiscard]] const CoveragePass * find_pass_for_job(
    const gpr_common::CoverageJob & job) const;
  [[nodiscard]] std::optional<gpr_common::Polyline> plan_astar_path(
    const gpr_common::Pose2D & from, const gpr_common::Pose2D & to) const;
  [[nodiscard]] bool is_job_reachable(
    const gpr_common::CoverageJob & job,
    const gpr_common::Pose2D & robot_pose,
    double transit_skip_distance) const;
  void filter_schedule_by_reachability(
    const gpr_common::Pose2D & robot_pose, double transit_skip_distance);
  void block_unreachable_schedulable(
    const gpr_common::Pose2D & robot_pose,
    const std::vector<gpr_common::CoverageSegment> & schedulable);
  void update_reachability_state(
    const gpr_common::Pose2D & robot_pose,
    const std::vector<gpr_common::CoverageSegment> & schedulable);
  void record_idle_schedule_state();

  mutable std::mutex state_mutex_;
  BoustrophedonPlanner planner_;
  PathInvalidatorConfig invalidator_config_;
  AStarConfig astar_config_;
  AStarGridPlanner astar_;
  std::unique_ptr<ISequenceSolver> solver_;
  SegmentCatalog catalog_;
  OarpLitePlanner oarp_planner_;
  BoustrophedonSequencer boustrophedon_sequencer_;
  bool use_boustrophedon_sequencer_{true};
  bool show_completed_in_markers_{false};
  gpr_common::Polyline initial_path_;
  gpr_common::GridMapConstPtr latest_grid_;
  std::uint64_t grid_seq_{0};
  gpr_common::Polyline executed_trace_;
  gpr_common::Polyline executed_coverage_trace_;
  gpr_common::Polyline executed_transit_trace_;
  gpr_common::Polyline mission_executed_trace_;
  gpr_common::SwathCoverageConfig swath_coverage_config_{};
  bool require_reachable_transit_{true};
  double transit_skip_distance_{0.35};
  double executed_trace_step_m_{0.05};
  double flush_pose_min_step_m_{1e-4};
  PlanningDiagnosticsConfig diagnostics_config_{};
  gpr_metrics::MetricsConfig metrics_config_{};
  std::optional<gpr_common::Pose2D> route_end_pose_;
  std::vector<CoveragePass> scheduled_passes_;
  std::unordered_map<gpr_common::SegmentId, std::size_t> pass_index_by_job_id_;
  std::vector<gpr_common::CoverageJob> schedule_;
  std::optional<gpr_common::CoverageJob> current_job_;
  gpr_common::Polyline active_transit_path_;
  std::optional<gpr_common::SegmentId> active_transit_job_id_;
  gpr_common::Polyline last_executed_path_;
  std::uint64_t schedule_revision_{0};
  std::uint64_t last_catalog_revision_{0};
  bool catalog_changed_{false};
  bool last_reachability_valid_{false};
  bool last_had_reachable_jobs_{false};
  std::uint64_t last_reachability_catalog_rev_{0};
  /// @brief Catalog revision when replan last yielded no executable jobs (anti-spin guard).
  std::uint64_t last_empty_schedule_catalog_rev_{0};
  bool grid_sync_logged_{false};
  std::string frame_id_;
  gpr_metrics::CoverageReporter reporter_;

  PlanningLog log_;
  std::int64_t last_update_grid_log_ns_{0};
  std::function<void()> viz_refresh_callback_;
};

}  // namespace gpr_planning

#endif  // GPR_PLANNING__PLANNING_ENGINE_HPP_
