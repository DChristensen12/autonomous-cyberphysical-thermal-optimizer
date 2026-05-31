// Pure PID math, no hardware anywhere in here. The whole point of pulling
// this out is that the laptop simulation and the real Teensy run the exact
// same controller, so if the sim says a set of gains is good, the hardware
// behaves the same way (modulo the physical model being right).
//
// No integral term on purpose. The GP is only tuning Kp and Kd, and tossing
// in an I term would muddy the performance score we feed back to it.

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    explicit PIDController(float setpoint = 0.0f)
        : kp_(0.0f),
          kd_(0.0f),
          setpoint_(setpoint),
          prev_error_(0.0f),
          last_error_(0.0f),
          first_tick_(true)
    {}

    void set_gains(float kp, float kd) {
        kp_ = kp;
        kd_ = kd;
        // Drop the derivative history. If we don't, the first step after a
        // gain change sees a huge bogus error delta and kicks the actuator.
        first_tick_ = true;
    }

    void set_setpoint(float sp) { setpoint_ = sp; }
    float setpoint() const { return setpoint_; }

    // Forget the past. Call this at the start of every trial so one trial
    // can't bleed derivative state into the next.
    void reset() {
        prev_error_ = 0.0f;
        last_error_ = 0.0f;
        first_tick_ = true;
    }

    // Run one step. Give it the measured value and the time since the last
    // call. Returns the raw control signal. Positive means heat, negative
    // means cool. The caller decides what to do with it.
    float compute(float measurement, float dt_s) {
        const float error = setpoint_ - measurement;
        last_error_ = error;

        float derivative = 0.0f;
        if (!first_tick_ && dt_s > 1e-6f) {
            derivative = (error - prev_error_) / dt_s;
        }
        first_tick_ = false;
        prev_error_ = error;

        return kp_ * error + kd_ * derivative;
    }

    float last_error() const { return last_error_; }
    float kp() const { return kp_; }
    float kd() const { return kd_; }

private:
    float kp_;
    float kd_;
    float setpoint_;
    float prev_error_;
    float last_error_;
    bool  first_tick_;
};

#endif
