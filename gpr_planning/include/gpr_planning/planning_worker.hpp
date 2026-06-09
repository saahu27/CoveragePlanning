#ifndef GPR_PLANNING__PLANNING_WORKER_HPP_
#define GPR_PLANNING__PLANNING_WORKER_HPP_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>

#include "gpr_planning/planning_snapshot.hpp"

namespace gpr_planning
{

struct PlanningServices;

/// @brief Dedicated planning thread: OR-Tools / schedule rebuild off the BT tick path.
class PlanningWorker
{
public:
  explicit PlanningWorker(const std::shared_ptr<PlanningServices> & services);
  ~PlanningWorker();

  PlanningWorker(const PlanningWorker &) = delete;
  PlanningWorker & operator=(const PlanningWorker &) = delete;

  void start();
  void stop();

  [[nodiscard]] std::uint64_t submit(PlanningJobRequest req);
  void cancel_pending_below(PlanningJobPriority max_priority);
  [[nodiscard]] std::optional<PlanningJobResult> poll(std::uint64_t request_id) const;
  [[nodiscard]] bool busy() const noexcept;

private:
  void worker_loop();
  PlanningJobResult run_job(const PlanningJobRequest & req);
  void store_result(PlanningJobResult result);

  std::shared_ptr<PlanningServices> services_;
  std::thread thread_;
  mutable std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<PlanningJobRequest> queue_;
  std::atomic<bool> stop_{false};
  std::atomic<std::uint64_t> next_request_id_{1};
  std::atomic<std::uint64_t> running_request_id_{0};

  mutable std::mutex result_mutex_;
  std::unordered_map<std::uint64_t, PlanningJobResult> results_;
};

}  // namespace gpr_planning

#endif  // GPR_PLANNING__PLANNING_WORKER_HPP_
