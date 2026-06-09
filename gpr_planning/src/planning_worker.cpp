#include "gpr_planning/planning_worker.hpp"

#include <algorithm>
#include <utility>

#include "gpr_planning/planning_ops.hpp"

namespace gpr_planning
{

PlanningWorker::PlanningWorker(const std::shared_ptr<PlanningServices> & services)
: services_(services)
{}

PlanningWorker::~PlanningWorker()
{
  stop();
}

void PlanningWorker::start()
{
  if (thread_.joinable()) {
    return;
  }
  stop_.store(false);
  thread_ = std::thread(&PlanningWorker::worker_loop, this);
}

void PlanningWorker::stop()
{
  stop_.store(true);
  queue_cv_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_.clear();
  }
}

std::uint64_t PlanningWorker::submit(PlanningJobRequest req)
{
  const std::uint64_t id = ++next_request_id_;
  req.request_id = id;

  std::lock_guard<std::mutex> lock(queue_mutex_);

  if (req.priority <= PlanningJobPriority::Normal) {
    queue_.erase(
      std::remove_if(
        queue_.begin(), queue_.end(),
        [](const PlanningJobRequest & pending) {
          return pending.priority <= PlanningJobPriority::Normal &&
                 pending.kind == PlanningJobKind::RecomputeSchedule;
        }),
      queue_.end());
  }

  if (req.priority >= PlanningJobPriority::Urgent) {
    queue_.push_front(std::move(req));
  } else {
    queue_.push_back(std::move(req));
  }

  queue_cv_.notify_one();
  return id;
}

void PlanningWorker::cancel_pending_below(const PlanningJobPriority max_priority)
{
  std::deque<PlanningJobRequest> cancelled;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    auto it = queue_.begin();
    while (it != queue_.end()) {
      if (it->priority <= max_priority) {
        cancelled.push_back(std::move(*it));
        it = queue_.erase(it);
      } else {
        ++it;
      }
    }
  }
  for (const auto & pending : cancelled) {
    PlanningJobResult result;
    result.request_id = pending.request_id;
    result.status = PlanningJobStatus::Cancelled;
    result.input_catalog_rev = pending.snapshot.catalog_rev;
    result.input_grid_seq = pending.snapshot.grid_seq;
    store_result(std::move(result));
  }
}

std::optional<PlanningJobResult> PlanningWorker::poll(const std::uint64_t request_id) const
{
  std::lock_guard<std::mutex> lock(result_mutex_);
  const auto it = results_.find(request_id);
  if (it == results_.end()) {
    if (running_request_id_.load() == request_id) {
      PlanningJobResult running;
      running.request_id = request_id;
      running.status = PlanningJobStatus::Running;
      return running;
    }
    PlanningJobResult pending;
    pending.request_id = request_id;
    pending.status = PlanningJobStatus::Pending;
    return pending;
  }
  return it->second;
}

bool PlanningWorker::busy() const noexcept
{
  if (running_request_id_.load() != 0U) {
    return true;
  }
  std::lock_guard<std::mutex> lock(queue_mutex_);
  return !queue_.empty();
}

void PlanningWorker::store_result(PlanningJobResult result)
{
  std::lock_guard<std::mutex> lock(result_mutex_);
  results_[result.request_id] = result;
}

PlanningJobResult PlanningWorker::run_job(const PlanningJobRequest & req)
{
  PlanningJobResult result;
  result.request_id = req.request_id;
  result.input_catalog_rev = req.snapshot.catalog_rev;
  result.input_grid_seq = req.snapshot.grid_seq;
  result.status = PlanningJobStatus::Running;
  store_result(result);

  if (!services_ || !services_->engine) {
    result.status = PlanningJobStatus::Failed;
    result.error = "planning services unavailable";
    store_result(result);
    return result;
  }

  auto & engine = *services_->engine;
  const std::uint64_t catalog_before = engine.catalog().revision();
  if (catalog_before != req.snapshot.catalog_rev && !req.force) {
    result.status = PlanningJobStatus::Superseded;
    store_result(result);
    return result;
  }

  if (req.snapshot.grid && !req.snapshot.grid->empty()) {
    apply_snapshot_grid(*services_, req.snapshot.grid, req.snapshot.grid_seq);
  }

  try {
    const bool has_work = recompute_and_publish(*services_, req.snapshot.robot_pose);
    result.output_schedule_rev = engine.schedule_revision();
    result.has_work = has_work;
    result.reachability_exhausted = engine.reachability_exhausted();

    const std::uint64_t catalog_after = engine.catalog().revision();
    if (catalog_after != req.snapshot.catalog_rev && !req.force) {
      result.status = PlanningJobStatus::Superseded;
    } else {
      result.status = PlanningJobStatus::Succeeded;
    }
  } catch (const std::exception & ex) {
    result.status = PlanningJobStatus::Failed;
    result.error = ex.what();
  }

  store_result(result);
  return result;
}

void PlanningWorker::worker_loop()
{
  while (!stop_.load()) {
    PlanningJobRequest req;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(
        lock, [this]() {
          return stop_.load() || !queue_.empty();
        });
      if (stop_.load() && queue_.empty()) {
        break;
      }
      if (queue_.empty()) {
        continue;
      }
      req = std::move(queue_.front());
      queue_.pop_front();
    }

    running_request_id_.store(req.request_id);
    (void)run_job(req);
    running_request_id_.store(0U);
  }
}

}  // namespace gpr_planning
