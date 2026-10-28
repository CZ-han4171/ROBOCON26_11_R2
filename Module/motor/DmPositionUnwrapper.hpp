#pragma once

// The DM position feedback is encoded in [-12.5, 12.5] rad. Keep a continuous
// motor-side angle while consecutive feedback frames differ by less than half
// of the 25 rad encoding span.
class DmPositionUnwrapper {
public:
  float update(float wrapped_pos_rad) {
    if (!initialized_) {
      initialized_ = true;
      previous_wrapped_pos_rad_ = wrapped_pos_rad;
      continuous_pos_rad_ = wrapped_pos_rad;
      return continuous_pos_rad_;
    }

    float delta = wrapped_pos_rad - previous_wrapped_pos_rad_;
    if (delta > kHalfRangeRad) {
      delta -= kRangeRad;
    } else if (delta < -kHalfRangeRad) {
      delta += kRangeRad;
    }
    continuous_pos_rad_ += delta;
    previous_wrapped_pos_rad_ = wrapped_pos_rad;
    return continuous_pos_rad_;
  }

private:
  static constexpr float kRangeRad = 25.0f;
  static constexpr float kHalfRangeRad = kRangeRad / 2.0f;
  bool initialized_{false};
  float previous_wrapped_pos_rad_{0.0f};
  float continuous_pos_rad_{0.0f};
};
