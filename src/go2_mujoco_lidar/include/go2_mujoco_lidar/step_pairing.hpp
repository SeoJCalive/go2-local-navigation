#pragma once
// Join DDS streams at identical simulation nanoseconds (LowState tick is ms).
// Keep bounded pending input and one latest complete pair, consumed once.
// A backwards stream clears the epoch and blocks old input from its peer
// until that stream also moves backwards.
#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <utility>

namespace go2_mujoco_lidar
{
template<typename Low, typename Sport>
class StepPairing
{
public:
  struct Pair
  {
    std::shared_ptr<const Low> low;
    std::shared_ptr<const Sport> sport;
  };

  bool add_low(std::int64_t stamp, std::shared_ptr<const Low> message)
  {
    const auto status = observe(0, stamp);
    if (status.first) {
      pending_[stamp].low = std::move(message);
      complete(stamp);
    }
    return status.second;
  }

  bool add_sport(std::int64_t stamp, std::shared_ptr<const Sport> message)
  {
    const auto status = observe(1, stamp);
    if (status.first) {
      pending_[stamp].sport = std::move(message);
      complete(stamp);
    }
    return status.second;
  }

  std::optional<Pair> take()
  {
    auto result = std::move(ready_);
    ready_.reset();
    return result;
  }

  std::size_t dropped() const {return dropped_;}
  std::size_t matched() const {return matched_;}
  std::size_t pending_size() const {return pending_.size();}

private:
  std::pair<bool, bool> observe(int stream, std::int64_t stamp)
  {
    const bool backwards = last_[stream] && stamp < *last_[stream];
    const bool duplicate = last_[stream] && stamp == *last_[stream];
    last_[stream] = stamp;
    bool reset = false;
    if (backwards) {
      if (awaiting_reset_ == stream) {
        awaiting_reset_ = -1;
      } else {
        pending_.clear();
        ready_.reset();
        completed_stamp_.reset();
        awaiting_reset_ = last_[1 - stream] ? 1 - stream : -1;
        reset = true;
      }
    }
    return {!duplicate && awaiting_reset_ != stream &&
      (!completed_stamp_ || stamp > *completed_stamp_), reset};
  }

  void complete(std::int64_t stamp)
  {
    auto & pair = pending_.at(stamp);
    if (pair.low && pair.sport) {
      ready_ = pair;
      completed_stamp_ = stamp;
      ++matched_;
      pending_.erase(pending_.begin(), pending_.upper_bound(stamp));
    }
    while (pending_.size() > 128) {
      pending_.erase(pending_.begin());
      ++dropped_;
    }
  }

  std::map<std::int64_t, Pair> pending_;
  std::array<std::optional<std::int64_t>, 2> last_{};
  std::optional<std::int64_t> completed_stamp_;
  std::optional<Pair> ready_;
  int awaiting_reset_{-1};
  std::size_t dropped_{0};
  std::size_t matched_{0};
};
}
