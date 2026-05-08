//! A proportional-integral-derivative (PID) controller library.
//!
//! See [Pid] for the adjustable controller itself, as well as [ControlOutput] for the outputs and weights which you can use after setting up your controller. Follow the complete example below to setup your first controller!
//!
//! # Example
//!
//! ```rust
//! use pid::Pid;
//!
//! // Create a new proportional-only PID controller with a setpoint of 15
//! let mut pid = Pid::new(15.0, 100.0);
//! pid.p(10.0, 100.0);
//!
//! // Input a measurement with an error of 5.0 from our setpoint
//! let output = pid.next_control_output(10.0);
//!
//! // Show that the error is correct by multiplying by our kp
//! assert_eq!(output.output, 50.0); // <--
//! assert_eq!(output.p, 50.0);
//!
//! // It won't change on repeat; the controller is proportional-only
//! let output = pid.next_control_output(10.0);
//! assert_eq!(output.output, 50.0); // <--
//! assert_eq!(output.p, 50.0);
//!
//! // Add a new integral term to the controller and input again
//! pid.i(1.0, 100.0);
//! let output = pid.next_control_output(10.0);
//!
//! // Now that the integral makes the controller stateful, it will change
//! assert_eq!(output.output, 55.0); // <--
//! assert_eq!(output.p, 50.0);
//! assert_eq!(output.i, 5.0);
//!
//! // Add our final derivative term and match our setpoint target
//! pid.d(2.0, 100.0);
//! let output = pid.next_control_output(15.0);
//!
//! // The output will now say to go down due to the derivative
//! assert_eq!(output.output, -5.0); // <--
//! assert_eq!(output.p, 0.0);
//! assert_eq!(output.i, 5.0);
//! assert_eq!(output.d, -10.0);
//! ```
#![no_std]

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg(all(feature = "defmt", not(test)))]
macro_rules! log_error {
    ($($arg:tt)*) => {{
        defmt::error!($($arg)*);
    }};
}

#[cfg(any(not(feature = "defmt"), test))]
macro_rules! log_error {
    ($($arg:tt)*) => {{}};
}

#[cfg(all(feature = "defmt", not(test)))]
macro_rules! log_warn {
    ($($arg:tt)*) => {{
        defmt::warn!($($arg)*);
    }};
}

#[cfg(any(not(feature = "defmt"), test))]
macro_rules! log_warn {
    ($($arg:tt)*) => {{}};
}

/// A trait for any numeric type usable in the PID controller
///
/// This trait is automatically implemented for all types that satisfy `PartialOrd + num_traits::Signed + Copy`. This includes all of the signed float types and builtin integer except for [isize]:
/// - [i8]
/// - [i16]
/// - [i32]
/// - [i64]
/// - [i128]
/// - [f32]
/// - [f64]
///
/// As well as any user type that matches the requirements
pub trait Number: PartialOrd + num_traits::Signed + Copy {}

// Implement `Number` for all types that
// satisfy `PartialOrd + num_traits::Signed + Copy`.
impl<T: PartialOrd + num_traits::Signed + Copy> Number for T {}

/// Condition function used by anti-windup helpers.
///
/// The arguments are the predicted output before the integral update, the
/// integral lower limit, the integral upper limit, and the current error.
pub type AntiWindupCondition<T> = fn(T, T, T, T) -> bool;

/// Adjustable proportional-integral-derivative (PID) controller.
///
/// # Examples
///
/// This controller provides a builder pattern interface which allows you to pick-and-choose which PID inputs you'd like to use during operation. Here's what a basic proportional-only controller could look like:
///
/// ```rust
/// use pid::Pid;
///
/// // Create limited controller
/// let mut p_controller = Pid::new(15.0, 100.0);
/// p_controller.p(10.0, 100.0);
///
/// // Get first output
/// let p_output = p_controller.next_control_output(400.0);
/// ```
///
/// This controller would give you set a proportional controller to `10.0` with a target of `15.0` and an output limit of `100.0` per [output](Self::next_control_output) iteration. The same controller with a full PID system built in looks like:
///
/// ```rust
/// use pid::Pid;
///
/// // Create full PID controller
/// let mut full_controller = Pid::new(15.0, 100.0);
/// full_controller.p(10.0, 100.0).i(4.5, 100.0).d(0.25, 100.0);
///
/// // Get first output
/// let full_output = full_controller.next_control_output(400.0);
/// ```
///
/// This [`next_control_output`](Self::next_control_output) method is what's used to input new values into the controller to tell it what the current state of the system is. In the examples above it's only being used once, but realistically this will be a hot method. Please see [ControlOutput] for examples of how to handle these outputs; it's quite straight forward and mirrors the values of this structure in some ways.
///
/// The last item of note is that these [`p`](Self::p()), [`i`](Self::i()), and [`d`](Self::d()) methods can be used *during* operation which lets you add and/or modify these controller values if need be.
///
/// # Type Warning
///
/// [Number] is abstract and can be used with anything from a [i32] to an [i128] (as well as user-defined types). Because of this, very small types might overflow during calculation in [`next_control_output`](Self::next_control_output). You probably don't want to use [i8] or user-defined types around that size so keep that in mind when designing your controller.
#[allow(unpredictable_function_pointer_comparisons)]
#[derive(Clone, Copy, Eq, PartialEq, Ord, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Pid<T: Number> {
    /// Ideal setpoint to strive for.
    pub setpoint: T,
    /// Defines the overall output filter limit.
    pub output_limit: T,
    /// Proportional gain.
    pub kp: T,
    /// Integral gain.
    pub ki: T,
    /// Derivative gain.
    pub kd: T,
    /// Limiter for the proportional term: `-p_limit <= P <= p_limit`.
    pub p_limit: T,
    /// Lower limit for the integral term: `i_min <= I`.
    pub i_min: T,
    /// Upper limit for the integral term: `I <= i_max`.
    pub i_max: T,
    /// Symmetric integral limit retained for compatibility.
    ///
    /// Calls to [Self::i()] set this to the absolute integral limit. Calls to
    /// [Self::i2()] set this to `max(i_min.abs(), i_max.abs())`.
    pub i_limit: T,
    /// Limiter for the derivative term: `-d_limit <= D <= d_limit`.
    pub d_limit: T,
    /// Last calculated integral value if [Pid::ki] is used.
    integral_term: T,
    /// Previously found measurement whilst using the [Pid::next_control_output] method.
    prev_measurement: Option<T>,
    /// Function that controls whether the next error is added to the integral term.
    #[cfg_attr(feature = "serde", serde(skip))]
    conditional_integration: Option<AntiWindupCondition<T>>,
     /// Integrator leak multiplier for anti-windup.
    integrator_leak: Option<T>,
    /// Function that controls whether the integrator leak is applied.
    #[cfg_attr(feature = "serde", serde(skip))]
    integrator_leak_condition: Option<AntiWindupCondition<T>>,
    /// Tracking time constant used for back-calculation anti-windup.
    tt: Option<T>,
}

/// Output of [controller iterations](Pid::next_control_output) with weights
///
/// # Example
///
/// This structure is simple to use and features three weights: [p](Self::p), [i](Self::i), and [d](Self::d). These can be used to figure out how much each term from [Pid] contributed to the final [output](Self::output) value which should be taken as the final controller output for this iteration:
///
/// ```rust
/// use pid::{Pid, ControlOutput};
///
/// // Setup controller
/// let mut pid = Pid::new(15.0, 100.0);
/// pid.p(10.0, 100.0).i(1.0, 100.0).d(2.0, 100.0);
///
/// // Input an example value and get a report for an output iteration
/// let output = pid.next_control_output(26.2456);
/// println!("P: {}\nI: {}\nD: {}\nFinal Output: {}", output.p, output.i, output.d, output.output);
/// ```
#[derive(Debug, PartialEq, Eq)]
pub struct ControlOutput<T: Number> {
    /// Contribution of the P term to the output.
    pub p: T,
    /// Contribution of the I term to the output.
    ///
    /// This integral term is equal to `sum[error(t) * ki(t)] (for all t)`
    pub i: T,
    /// Contribution of the D term to the output.
    pub d: T,
    /// Output of the PID controller.
    pub output: T,
}

impl<T> Pid<T>
where
    T: Number,
{
    /// Creates a new controller with the target setpoint and the output limit
    ///
    /// To set your P, I, and D terms into this controller, please use the following builder methods:
    /// - [Self::p()]: Proportional term setting
    /// - [Self::i()]: Integral term setting
    /// - [Self::d()]: Derivative term setting
    pub fn new(setpoint: impl Into<T>, output_limit: impl Into<T>) -> Self {
        Self {
            setpoint: setpoint.into(),
            output_limit: output_limit.into(),
            kp: T::zero(),
            ki: T::zero(),
            kd: T::zero(),
            p_limit: T::zero(),
            i_min: T::zero(),
            i_max: T::zero(),
            i_limit: T::zero(),
            d_limit: T::zero(),
            integral_term: T::zero(),
            prev_measurement: None,
            conditional_integration: None,
            integrator_leak: None,
            integrator_leak_condition: None,
            tt: None,
        }
    }

    /// Sets the [Self::p] term for this controller.
    pub fn p(&mut self, gain: impl Into<T>, limit: impl Into<T>) -> &mut Self {
        self.kp = gain.into();
        self.p_limit = limit.into();
        self
    }

    /// Sets the [Self::i] term with a symmetric integral limit.
    ///
    /// The integral term is clamped so that `-limit <= I <= limit`. The public
    /// [Self::i_limit] field is kept for backwards compatibility and can still
    /// be changed directly to update this symmetric limit.
    pub fn i(&mut self, gain: impl Into<T>, limit: impl Into<T>) -> &mut Self {
        self.ki = gain.into();
        let limit_val: T = limit.into().abs();
        self.i_min = -limit_val;
        self.i_max = limit_val;
        self.i_limit = limit_val;
        self
    }

    /// Sets the [Self::i] term with asymmetric minimum and maximum limits.
    ///
    /// The integral term is clamped so that `min <= I <= max`. If `min` is
    /// larger than `max`, the controller is left unchanged.
    pub fn i2(&mut self, gain: impl Into<T>, min: impl Into<T>, max: impl Into<T>) -> &mut Self {
        let i_min = min.into();
        let i_max = max.into();
        if i_min > i_max {
            log_error!("min > max");
            log_error!("Unable to set integral minimum and maximum");
            return self;
        }
        self.ki = gain.into();
        self.i_min = i_min;
        self.i_max = i_max;
        self.i_limit = if i_min.abs() > i_max.abs() {
            i_min.abs()
        } else {
            i_max.abs()
        };
        self
    }

    /// Sets the [Self::d] term for this controller.
    pub fn d(&mut self, gain: impl Into<T>, limit: impl Into<T>) -> &mut Self {
        self.kd = gain.into();
        self.d_limit = limit.into();
        self
    }

    /// Sets the [Pid::setpoint] to target for this controller.
    pub fn setpoint(&mut self, setpoint: impl Into<T>) -> &mut Self {
        self.setpoint = setpoint.into();
        self
    }

    /// Enables conditional integration for the integral term.
    ///
    /// The provided function decides whether the current error should
    /// contribute to the integral term. If the condition evaluates to `true`,
    /// the error is added; otherwise, the integral is left unchanged.
    ///
    /// The function signature is:
    ///
    /// `fn(u_pred, i_min, i_max, e) -> bool`
    ///
    /// - `u_pred`: predicted controller output *before* updating the integral term
    /// - `i_min`: lower bound of the integral term
    /// - `i_max`: upper bound of the integral term
    /// - `e`: current error (setpoint - measurement)
    ///
    /// # Examples
    ///
    /// 1. Prevent integration while saturating:
    ///    ```ignore
    ///    fn(u_pred, i_min, i_max, e) -> bool {
    ///        !(u_pred >= i_max && e > 0.0) && !(u_pred <= i_min && e < 0.0)
    ///    }
    ///    ```
    ///
    /// 2. Integrate only for small errors:
    ///    ```ignore
    ///    fn(_u_pred, _i_min, _i_max, e) -> bool {
    ///        e.abs() < 10.0
    ///    }
    ///    ```
    pub fn aw_conditional_integration(&mut self, fun: AntiWindupCondition<T>) -> &mut Self {
        if self.tt.is_some() {
            log_warn!("Unable to use conditional integration with back-calculation.");
            log_warn!("Disabled back-calculation.");
            self.tt = None;
        }
        self.conditional_integration = Some(fun);
        self
    }

    /// Enables integrator leaking for the integral term.
    ///
    /// On every controller iteration, the current integral term is multiplied
    /// by `leak_rate` before the new error is integrated. A value of `1` keeps
    /// the integrator unchanged, while `0` clears it on every iteration. Values
    /// outside `0..=1` are rejected and leave the controller unchanged.
    ///
    /// Integrator leak cannot be combined with back-calculation. Enabling this
    /// method disables a previously configured back-calculation mode.
    pub fn aw_integrator_leak(&mut self, leak_rate: T) -> &mut Self {
        if leak_rate < T::zero() || leak_rate > T::one() {
            log_error!("Leak rate must be between 0 and 1!");
            log_error!("Integrator leak not set.");
            self
        } else {
            if self.tt.is_some() {
                log_warn!("Unable to use integrator leak with back-calculation.");
                log_warn!("Disabled back-calculation.");
                self.tt = None;
            }
            self.integrator_leak = Some(leak_rate);
            self.integrator_leak_condition = None;
            self
        }
    }

    /// Enables integrator leaking only when a condition function returns `true`.
    ///
    /// The function receives the same values as
    /// [Self::aw_conditional_integration()]: the predicted output before the
    /// integral update, the integral minimum, the integral maximum, and the
    /// current error. If it returns `true`, the current integral term is
    /// multiplied by `leak_rate`; otherwise the leak is skipped for that
    /// iteration.
    ///
    /// Conditional integrator leak cannot be combined with back-calculation.
    /// Enabling this method disables a previously configured back-calculation
    /// mode.
    pub fn aw_conditional_integrator_leak(
        &mut self,
        leak_rate: T,
        fun: AntiWindupCondition<T>,
    ) -> &mut Self {
        if leak_rate < T::zero() || leak_rate > T::one() {
            log_error!("Leak rate must be between 0 and 1!");
            log_error!("Integrator leak not set.");
            self
        } else {
            if self.tt.is_some() {
                log_warn!("Unable to use integrator leak with back-calculation.");
                log_warn!("Disabled back-calculation.");
                self.tt = None;
            }
            self.integrator_leak = Some(leak_rate);
            self.integrator_leak_condition = Some(fun);
            self
        }
    }

    /// Enables back-calculation anti-windup for the integral term.
    ///
    /// Back-calculation feeds the difference between the saturated output and
    /// the unconstrained output back into the integral term. This lets the
    /// integrator recover while the final output is saturated by
    /// [Self::output_limit].
    ///
    /// Pass `Some(tt)` to set the tracking time constant explicitly. Passing
    /// `None` uses `kp / ki`, so `ki` must be non-zero. The tracking time
    /// constant must be positive; invalid values are rejected and leave the
    /// controller unchanged.
    ///
    /// Back-calculation cannot be combined with conditional integration or
    /// integrator leak. Enabling it disables those modes.
    pub fn aw_back_calculation(&mut self, tt: Option<T>) -> &mut Self {
        let tt = match tt {
            Some(tt) => tt,
            None if self.ki == T::zero() => {
                log_error!("Unable to derive back-calculation tracking time with zero ki.");
                log_error!("Back-calculation not set.");
                return self;
            }
            None => self.kp / self.ki,
        };

        if tt <= T::zero() {
            log_error!("Back-calculation tracking time must be greater than zero.");
            log_error!("Back-calculation not set.");
            return self;
        }

        if self.conditional_integration.is_some() {
            log_warn!("Unable to use conditional integration with back-calculation.");
            log_warn!("Disabled conditional integration.");
            self.conditional_integration = None;
        }
        if self.integrator_leak.is_some() {
            log_warn!("Unable to use integrator leak with back-calculation.");
            log_warn!("Disabled integrator leak.");
            self.integrator_leak = None;
            self.integrator_leak_condition = None;
        }
        self.tt = Some(tt);
        self
    }

    /// Given a new measurement, calculates the next [control output](ControlOutput).
    pub fn next_control_output(&mut self, measurement: T) -> ControlOutput<T> {
        // Calculate the error between the ideal setpoint and the current
        // measurement to compare against
        let error = self.setpoint - measurement;

        // Calculate the proportional term and limit to it's individual limit
        let p_unbounded = error * self.kp;
        let p = apply_limit(self.p_limit, p_unbounded);

        // Mitigate derivative kick: Use the derivative of the measurement
        // rather than the derivative of the error.
        let d_unbounded = -match self.prev_measurement.as_ref() {
            Some(prev_measurement) => measurement - *prev_measurement,
            None => T::zero(),
        } * self.kd;
        self.prev_measurement = Some(measurement);
        let d = apply_limit(self.d_limit, d_unbounded);

        let pred_output = p + self.integral_term + d;

        if let Some(tt) = self.tt {
            let integral_unbounded = self.integral_term + error * self.ki;
            let output_unbounded = p + integral_unbounded + d;
            let output_limited = apply_limit(self.output_limit, output_unbounded);
            let tracking = (output_limited - output_unbounded) / tt;
            self.integral_term = integral_unbounded + tracking;
        } else {
            if let Some(integrator_leak) = self.integrator_leak {
                let should_leak = match self.integrator_leak_condition {
                    Some(fun) => fun(pred_output, self.i_min, self.i_max, error),
                    None => true,
                };

                if should_leak {
                    self.integral_term = integrator_leak * self.integral_term;
                }
            }

            // Only add the error term to the integral if the condition function
            // returns true, or no condition function is set.
            if match self.conditional_integration {
                Some(fun) => fun(pred_output, self.i_min, self.i_max, error),
                None => true,
            } {
                // Mitigate output jumps when ki(t) != ki(t-1).
                // While it's standard to use an error_integral that's a running
                // sum of just the error (no ki), because we support ki changing
                // dynamically, we store the entire term so that we don't need to
                // remember previous ki values.
                self.integral_term = self.integral_term + error * self.ki;
            }
        }

        // Mitigate integral windup: Don't want to keep building up error
        // beyond what the configured integral limits will allow. If the public
        // i_limit field was changed directly, keep the original symmetric-limit
        // behavior for backwards compatibility.
        let configured_i_limit = if self.i_min.abs() > self.i_max.abs() {
            self.i_min.abs()
        } else {
            self.i_max.abs()
        };
        let (i_min, i_max) = if self.i_limit.abs() != configured_i_limit {
            let i_limit = self.i_limit.abs();
            (-i_limit, i_limit)
        } else {
            (self.i_min, self.i_max)
        };
        self.integral_term = apply_limit2(i_min, i_max, self.integral_term);

        // Calculate the final output by adding together the PID terms, then
        // apply the final defined output limit
        let output = p + self.integral_term + d;
        let output = apply_limit(self.output_limit, output);

        // Return the individual term's contributions and the final output
        ControlOutput {
            p,
            i: self.integral_term,
            d,
            output,
        }
    }

    /// Resets the integral term back to zero, this may drastically change the
    /// control output.
    pub fn reset_integral_term(&mut self) {
        self.integral_term = T::zero();
    }

    /// Set integral term to custom value.
    /// This may drastically change the control output.
    /// Use this to return the PID controller to a previous state after an interruption or crash.
    pub fn set_integral_term(&mut self, integral_term: impl Into<T>) -> &mut Self {
        self.integral_term = integral_term.into();
        self
    }

    /// Get the integral term.
    pub fn get_integral_term(&self) -> T {
        self.integral_term
    }
}

/// Saturating the input `value` according the absolute `limit` (`-abs(limit) <= output <= abs(limit)`).
fn apply_limit<T: Number>(limit: T, value: T) -> T {
    num_traits::clamp(value, -limit.abs(), limit.abs())
}

/// Saturating the input `value` between `min` and `max` (`min <= output <= max`).
fn apply_limit2<T: Number>(min: T, max: T, value: T) -> T {
    num_traits::clamp(value, min, max)
}

#[cfg(test)]
mod tests {
    use super::Pid;
    use crate::ControlOutput;

    fn never_integrate(_u_pred: f64, _i_min: f64, _i_max: f64, _error: f64) -> bool {
        false
    }

    fn leak_when_error_positive(_u_pred: f64, _i_min: f64, _i_max: f64, error: f64) -> bool {
        error > 0.0
    }

    /// Proportional-only controller operation and limits
    #[test]
    fn proportional() {
        let mut pid = Pid::new(10.0, 100.0);
        pid.p(2.0, 100.0).i(0.0, 100.0).d(0.0, 100.0);
        assert_eq!(pid.setpoint, 10.0);

        // Test simple proportional
        assert_eq!(pid.next_control_output(0.0).output, 20.0);

        // Test proportional limit
        pid.p_limit = 10.0;
        assert_eq!(pid.next_control_output(0.0).output, 10.0);
    }

    /// Derivative-only controller operation and limits
    #[test]
    fn derivative() {
        let mut pid = Pid::new(10.0, 100.0);
        pid.p(0.0, 100.0).i(0.0, 100.0).d(2.0, 100.0);

        // Test that there's no derivative since it's the first measurement
        assert_eq!(pid.next_control_output(0.0).output, 0.0);

        // Test that there's now a derivative
        assert_eq!(pid.next_control_output(5.0).output, -10.0);

        // Test derivative limit
        pid.d_limit = 5.0;
        assert_eq!(pid.next_control_output(10.0).output, -5.0);
    }

    /// Integral-only controller operation and limits
    #[test]
    fn integral() {
        let mut pid = Pid::new(10.0, 100.0);
        pid.p(0.0, 100.0).i(2.0, 100.0).d(0.0, 100.0);

        // Test basic integration
        assert_eq!(pid.next_control_output(0.0).output, 20.0);
        assert_eq!(pid.next_control_output(0.0).output, 40.0);
        assert_eq!(pid.next_control_output(5.0).output, 50.0);

        // Test limit
        pid.i_limit = 50.0;
        assert_eq!(pid.next_control_output(5.0).output, 50.0);
        // Test that limit doesn't impede reversal of error integral
        assert_eq!(pid.next_control_output(15.0).output, 40.0);

        // Test that error integral accumulates negative values
        let mut pid2 = Pid::new(-10.0, 100.0);
        pid2.p(0.0, 100.0).i(2.0, 100.0).d(0.0, 100.0);
        assert_eq!(pid2.next_control_output(0.0).output, -20.0);
        assert_eq!(pid2.next_control_output(0.0).output, -40.0);

        pid2.i_limit = 50.0;
        assert_eq!(pid2.next_control_output(-5.0).output, -50.0);
        // Test that limit doesn't impede reversal of error integral
        assert_eq!(pid2.next_control_output(-15.0).output, -40.0);
    }

    /// Asymmetric integral limits clamp both sides independently.
    #[test]
    fn integral_asymmetric_limits() {
        let mut pid = Pid::new(10.0, 100.0);
        pid.p(0.0, 100.0).i2(2.0, -10.0, 30.0).d(0.0, 100.0);

        assert_eq!(pid.i_min, -10.0);
        assert_eq!(pid.i_max, 30.0);
        assert_eq!(pid.i_limit, 30.0);
        assert_eq!(pid.next_control_output(0.0).i, 20.0);
        assert_eq!(pid.next_control_output(0.0).i, 30.0);
        assert_eq!(pid.next_control_output(20.0).i, 10.0);
        assert_eq!(pid.next_control_output(20.0).i, -10.0);
        assert_eq!(pid.next_control_output(20.0).i, -10.0);
    }

    /// Invalid asymmetric limits should not change the existing integral settings.
    #[test]
    fn invalid_asymmetric_limits_are_ignored() {
        let mut pid: Pid<f64> = Pid::new(10.0, 100.0);
        pid.p(0.0, 100.0).i(1.0, 5.0).d(0.0, 100.0);

        pid.i2(2.0, 10.0, -10.0);

        assert_eq!(pid.ki, 1.0);
        assert_eq!(pid.i_min, -5.0);
        assert_eq!(pid.i_max, 5.0);
        assert_eq!(pid.i_limit, 5.0);
    }

    /// Conditional integration skips accumulation when its condition returns false.
    #[test]
    fn anti_windup_conditional_integration() {
        let mut pid = Pid::new(10.0, 100.0);
        pid.p(0.0, 100.0)
            .i(1.0, 100.0)
            .d(0.0, 100.0)
            .aw_conditional_integration(never_integrate);

        assert_eq!(pid.next_control_output(0.0).i, 0.0);
        assert_eq!(pid.next_control_output(0.0).i, 0.0);
    }

    /// Integrator leak decays the stored integral before the next integration step.
    #[test]
    fn anti_windup_integrator_leak() {
        let mut pid = Pid::new(10.0, 100.0);
        pid.p(0.0, 100.0)
            .i(1.0, 100.0)
            .d(0.0, 100.0)
            .aw_integrator_leak(0.5);

        assert_eq!(pid.next_control_output(0.0).i, 10.0);

        pid.setpoint(0.0);

        assert_eq!(pid.next_control_output(0.0).i, 5.0);
    }

    /// Conditional integrator leak applies only when its condition returns true.
    #[test]
    fn anti_windup_conditional_integrator_leak() {
        let mut pid = Pid::new(10.0, 100.0);
        pid.p(0.0, 100.0)
            .i(1.0, 100.0)
            .d(0.0, 100.0)
            .aw_conditional_integrator_leak(0.5, leak_when_error_positive);

        assert_eq!(pid.next_control_output(0.0).i, 10.0);

        pid.setpoint(0.0);
        assert_eq!(pid.next_control_output(0.0).i, 10.0);

        pid.setpoint(10.0);
        assert_eq!(pid.next_control_output(0.0).i, 15.0);
    }

    /// Back-calculation feeds output saturation back into the integral term.
    #[test]
    fn anti_windup_back_calculation() {
        let mut pid = Pid::new(10.0, 10.0);
        pid.p(0.0, 100.0)
            .i(1.0, 100.0)
            .d(0.0, 100.0)
            .aw_back_calculation(Some(2.0));

        let out = pid.next_control_output(0.0);
        assert_eq!(out.i, 10.0);
        assert_eq!(out.output, 10.0);

        let out = pid.next_control_output(0.0);
        assert_eq!(out.i, 15.0);
        assert_eq!(out.output, 10.0);
    }

    /// Back-calculation and conditional integration are mutually exclusive.
    #[test]
    fn back_calculation_disables_conditional_integration() {
        let mut pid = Pid::new(10.0, 10.0);
        pid.p(0.0, 100.0)
            .i(1.0, 100.0)
            .d(0.0, 100.0)
            .aw_conditional_integration(never_integrate)
            .aw_back_calculation(Some(2.0));

        assert_eq!(pid.next_control_output(0.0).i, 10.0);
    }

    /// Enabling integrator leak disables a previously configured back-calculation mode.
    #[test]
    fn integrator_leak_disables_back_calculation() {
        let mut pid = Pid::new(10.0, 10.0);
        pid.p(0.0, 100.0)
            .i(1.0, 100.0)
            .d(0.0, 100.0)
            .aw_back_calculation(Some(2.0))
            .aw_integrator_leak(0.5);

        assert_eq!(pid.next_control_output(0.0).i, 10.0);

        pid.setpoint(0.0);

        assert_eq!(pid.next_control_output(0.0).i, 5.0);
    }

    /// Checks that a full PID controller's limits work properly through multiple output iterations
    #[test]
    fn output_limit() {
        let mut pid = Pid::new(10.0, 1.0);
        pid.p(1.0, 100.0).i(0.0, 100.0).d(0.0, 100.0);

        let out = pid.next_control_output(0.0);
        assert_eq!(out.p, 10.0); // 1.0 * 10.0
        assert_eq!(out.output, 1.0);

        let out = pid.next_control_output(20.0);
        assert_eq!(out.p, -10.0); // 1.0 * (10.0 - 20.0)
        assert_eq!(out.output, -1.0);
    }

    /// Combined PID operation
    #[test]
    fn pid() {
        let mut pid = Pid::new(10.0, 100.0);
        pid.p(1.0, 100.0).i(0.1, 100.0).d(1.0, 100.0);

        let out = pid.next_control_output(0.0);
        assert_eq!(out.p, 10.0); // 1.0 * 10.0
        assert_eq!(out.i, 1.0); // 0.1 * 10.0
        assert_eq!(out.d, 0.0); // -(1.0 * 0.0)
        assert_eq!(out.output, 11.0);

        let out = pid.next_control_output(5.0);
        assert_eq!(out.p, 5.0); // 1.0 * 5.0
        assert_eq!(out.i, 1.5); // 0.1 * (10.0 + 5.0)
        assert_eq!(out.d, -5.0); // -(1.0 * 5.0)
        assert_eq!(out.output, 1.5);

        let out = pid.next_control_output(11.0);
        assert_eq!(out.p, -1.0); // 1.0 * -1.0
        assert_eq!(out.i, 1.4); // 0.1 * (10.0 + 5.0 - 1)
        assert_eq!(out.d, -6.0); // -(1.0 * 6.0)
        assert_eq!(out.output, -5.6);

        let out = pid.next_control_output(10.0);
        assert_eq!(out.p, 0.0); // 1.0 * 0.0
        assert_eq!(out.i, 1.4); // 0.1 * (10.0 + 5.0 - 1.0 + 0.0)
        assert_eq!(out.d, 1.0); // -(1.0 * -1.0)
        assert_eq!(out.output, 2.4);
    }

    // NOTE: use for new test in future: /// Full PID operation with mixed float checking to make sure they're equal
    /// PID operation with zero'd values, checking to see if different floats equal each other
    #[test]
    fn floats_zeros() {
        let mut pid_f32 = Pid::new(10.0f32, 100.0);
        pid_f32.p(0.0, 100.0).i(0.0, 100.0).d(0.0, 100.0);

        let mut pid_f64 = Pid::new(10.0, 100.0f64);
        pid_f64.p(0.0, 100.0).i(0.0, 100.0).d(0.0, 100.0);

        for _ in 0..5 {
            assert_eq!(
                pid_f32.next_control_output(0.0).output,
                pid_f64.next_control_output(0.0).output as f32
            );
        }
    }

    /// Full PID operation with mixed signed integer checking to make sure they're equal
    /// PID operation with zero'd values, checking to see if different floats equal each other
    #[test]
    fn signed_integers() {
        let mut pid_i8 = Pid::new(10i8, 100);
        pid_i8.p(0, 100).i(0, 100).d(0, 100);

        let mut pid_i16 = Pid::new(10i16, 100i16);
        pid_i16.p(0i16, 100i16).i(0i16, 100i16).d(0i16, 100i16);

        let mut pid_i32 = Pid::new(10i32, 100);
        pid_i32.p(0, 100).i(0, 100).d(0, 100);

        let mut pid_i64 = Pid::new(10i64, 100);
        pid_i64.p(0, 100).i(0, 100).d(0, 100);

        let mut pid_i128 = Pid::new(10i128, 100);
        pid_i128.p(0, 100).i(0, 100).d(0, 100);

        for _ in 0..5 {
            assert_eq!(
                pid_i8.next_control_output(0i8).output as i16,
                pid_i16.next_control_output(0i16).output,
            );

            assert_eq!(
                pid_i16.next_control_output(0i16).output as i32,
                pid_i32.next_control_output(0i32).output,
            );

            assert_eq!(
                pid_i32.next_control_output(0i32).output as i64,
                pid_i64.next_control_output(0i64).output
            );

            assert_eq!(
                pid_i64.next_control_output(0i64).output as i128,
                pid_i128.next_control_output(0i128).output
            );
        }
    }

    #[test]
    fn float_match_integers() {
        let mut pid_i32 = Pid::new(10i32, 100);
        pid_i32.p(0, 100).i(0, 100).d(0, 100);

        let mut pid_f32 = Pid::new(10.0f32, 100.0);
        pid_f32.p(0.0, 100.0).i(0.0, 100.0).d(0.0, 100.0);

        for _ in 0..5 {
            assert_eq!(
                pid_i32.next_control_output(0i32).output as f32,
                pid_f32.next_control_output(0f32).output
            );
        }
    }

    /// See if the controller can properly target to the setpoint after 2 output iterations
    #[test]
    fn setpoint() {
        let mut pid = Pid::new(10.0, 100.0);
        pid.p(1.0, 100.0).i(0.1, 100.0).d(1.0, 100.0);

        let out = pid.next_control_output(0.0);
        assert_eq!(out.p, 10.0); // 1.0 * 10.0
        assert_eq!(out.i, 1.0); // 0.1 * 10.0
        assert_eq!(out.d, 0.0); // -(1.0 * 0.0)
        assert_eq!(out.output, 11.0);

        pid.setpoint(0.0);

        assert_eq!(
            pid.next_control_output(0.0),
            ControlOutput {
                p: 0.0,
                i: 1.0,
                d: -0.0,
                output: 1.0
            }
        );
    }

    /// Make sure negative limits don't break the controller
    #[test]
    fn negative_limits() {
        let mut pid = Pid::new(10.0f32, -10.0);
        pid.p(1.0, -50.0).i(1.0, -50.0).d(1.0, -50.0);

        let out = pid.next_control_output(0.0);
        assert_eq!(out.p, 10.0);
        assert_eq!(out.i, 10.0);
        assert_eq!(out.d, 0.0);
        assert_eq!(out.output, 10.0);
    }
}
