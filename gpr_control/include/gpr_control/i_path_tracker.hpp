#ifndef GPR_CONTROL__I_PATH_TRACKER_HPP_
#define GPR_CONTROL__I_PATH_TRACKER_HPP_

#include <functional>

#include "gpr_common/types.hpp"

namespace gpr_control
{

enum class TrackResult
{
  Succeeded,
  Aborted,
  Canceled,
};

using TrackCallback = std::function<void(TrackResult)>;

class IPathTracker
{
public:
  virtual ~IPathTracker() = default;
  virtual void follow(const gpr_common::Polyline & path, TrackCallback on_done) = 0;
  virtual void cancel() = 0;
};

}  // namespace gpr_control

#endif  // GPR_CONTROL__I_PATH_TRACKER_HPP_
