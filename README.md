# PID Controller for Rust
[![Latest Version]][crates.io] [![Documentation]][docs.rs] 

[Latest Version]: https://img.shields.io/crates/v/pid.svg
[crates.io]: https://crates.io/crates/pid
[Documentation]: https://docs.rs/pid/badge.svg
[docs.rs]: https://docs.rs/pid

A proportional-integral-derivative (PID) controller.

## Features

* Visibility into individual contribution of P, I, and D terms which often
  need to be logged for later analysis and parameter tuning.
* Output limits on a per term basis.
* Three-term control output limit.
* Mitigation of integral windup using integral term limit.
* Symmetric or asymmetric integral term limits.
* Additional anti-windup modes: conditional integration, integrator leak,
  conditional integrator leak, and back-calculation.
* Mitigation of derivative kick by using the derivative of the measurement
  rather than the derivative of the error.
* On-the-fly changes to `setpoint`/`kp`/`ki`/`kd`.
  * Mitigation of output jumps when changing `ki` by storing the integration of
    `e(t) * ki(t)` rather than only `e(t)`.
* Generic float type parameter to support `f32` or `f64`.
* Support for `no_std` environments, such as embedded systems.
* Optional support for [Serde](https://crates.io/crates/serde). Enable the
  `serde` Cargo feature, if you need `Pid` to implement
  `Serialize`/`Deserialize`.
* Optional support for [defmt](https://crates.io/crates/defmt). Enable the
  `defmt` Cargo feature to log invalid configuration warnings on embedded
  targets that support it.

## Cargo Features

This crate has no default features.

```toml
[dependencies]
pid = "4.1.0"
```

Enable `serde` support when `Pid` needs to be serialized or deserialized:

```toml
[dependencies]
pid = { version = "4.1.0", features = ["serde"] }
```

Enable `defmt` support when logging should be emitted through `defmt`:

```toml
[dependencies]
pid = { version = "4.1.0", features = ["defmt"] }
```

## Example

```rust
use pid::Pid;

// Create a new proportional-only PID controller with a setpoint of 15
let mut pid = Pid::new(15.0, 100.0);
pid.p(10.0, 100.0);

// Input a measurement with an error of 5.0 from our setpoint
let output = pid.next_control_output(10.0);

// Show that the error is correct by multiplying by our kp
assert_eq!(output.output, 50.0); // <--
assert_eq!(output.p, 50.0);

// It won't change on repeat; the controller is proportional-only
let output = pid.next_control_output(10.0);
assert_eq!(output.output, 50.0); // <--
assert_eq!(output.p, 50.0);

// Add a new integral term to the controller and input again
pid.i(1.0, 100.0);
let output = pid.next_control_output(10.0);

// Now that the integral makes the controller stateful, it will change
assert_eq!(output.output, 55.0); // <--
assert_eq!(output.p, 50.0);
assert_eq!(output.i, 5.0);

// Add our final derivative term and match our setpoint target
pid.d(2.0, 100.0);
let output = pid.next_control_output(15.0);

// The output will now say to go down due to the derivative
assert_eq!(output.output, -5.0); // <--
assert_eq!(output.p, 0.0);
assert_eq!(output.i, 5.0);
assert_eq!(output.d, -10.0);
```

## Anti-Windup Examples

The examples below use integral-only controllers to make the anti-windup effect
easy to see.

```rust
use pid::Pid;

// The original integral setup uses one symmetric limit: -limit <= I <= limit.
let mut pid = Pid::new(10.0, 100.0);
pid.i(2.0, 100.0);

// With an error of 10.0 and ki of 2.0, the integral term grows by 20.0.
assert_eq!(pid.next_control_output(0.0).i, 20.0);
assert_eq!(pid.next_control_output(0.0).i, 40.0);

// Backwards-compatible: changing i_limit directly still changes the symmetric
// integral limit.
pid.i_limit = 50.0;
assert_eq!(pid.next_control_output(0.0).i, 50.0);

// Use i2() when the integral term needs different lower and upper limits.
let mut pid = Pid::new(10.0, 100.0);
pid.i2(2.0, -10.0, 30.0);

assert_eq!(pid.next_control_output(0.0).i, 20.0);
assert_eq!(pid.next_control_output(0.0).i, 30.0);
```

Conditional integration lets you decide if the current error should be added to
the integral term.

```rust
use pid::Pid;

fn only_integrate_small_errors(
    _u_pred: f64,
    _i_min: f64,
    _i_max: f64,
    error: f64,
) -> bool {
    error.abs() < 5.0
}

let mut pid = Pid::new(10.0, 100.0);
pid.i(1.0, 100.0)
    .aw_conditional_integration(only_integrate_small_errors);

// Error is 10.0, so the condition blocks integration.
assert_eq!(pid.next_control_output(0.0).i, 0.0);

// Error is 2.0, so the condition allows integration.
assert_eq!(pid.next_control_output(8.0).i, 2.0);
```

Integrator leak decays the stored integral term before the next error is added.

```rust
use pid::Pid;

let mut pid = Pid::new(10.0, 100.0);
pid.i(1.0, 100.0).aw_integrator_leak(0.5);

// First iteration integrates the error normally.
assert_eq!(pid.next_control_output(0.0).i, 10.0);

// With zero error, only the leak remains: 10.0 * 0.5 = 5.0.
pid.setpoint(0.0);
assert_eq!(pid.next_control_output(0.0).i, 5.0);
```

Conditional integrator leak applies the leak only when its condition returns
`true`.

```rust
use pid::Pid;

fn leak_when_error_positive(
    _u_pred: f64,
    _i_min: f64,
    _i_max: f64,
    error: f64,
) -> bool {
    error > 0.0
}

let mut pid = Pid::new(10.0, 100.0);
pid.i(1.0, 100.0)
    .aw_conditional_integrator_leak(0.5, leak_when_error_positive);

assert_eq!(pid.next_control_output(0.0).i, 10.0);

// Error is zero, so the leak is skipped.
pid.setpoint(0.0);
assert_eq!(pid.next_control_output(0.0).i, 10.0);

// Error is positive, so the integral leaks before the new error is added.
pid.setpoint(10.0);
assert_eq!(pid.next_control_output(0.0).i, 15.0);
```

Back-calculation pulls the integral term back while the controller output is
saturated by `output_limit`.

```rust
use pid::Pid;

let mut pid = Pid::new(10.0, 10.0);
pid.i(1.0, 100.0).aw_back_calculation(Some(2.0));

let output = pid.next_control_output(0.0);
assert_eq!(output.i, 10.0);
assert_eq!(output.output, 10.0);

// The unconstrained output would be 20.0, but the output limit is 10.0.
// Back-calculation feeds half of that difference back into the integral term.
let output = pid.next_control_output(0.0);
assert_eq!(output.i, 15.0);
assert_eq!(output.output, 10.0);
```

Back-calculation is mutually exclusive with conditional integration and
integrator leak. Enabling one of these modes disables the conflicting mode.

## Assumptions

* Measurements occur at equal spacing. (`t(i) = t(i-1) + C`)
* Output limits for P and D terms are symmetric around 0
  (`-limit <= term <= limit`).
* Integral limits are symmetric when configured with `i()` and asymmetric when
  configured with `i2()`.

## Formulation

There are several different formulations of PID controllers. This library
uses the independent form:

```math
C(t) = K_p \cdot e(t) + K_i \cdot \int{e(t)dt} - K_d \cdot \frac{dP(t)}{dt}
```

Where:

- C(t) = control output, the output to the actuator.
- P(t) = process variable, the measured value.
- e(t) = error = S(t) - P(t)
- S(t) = set point, the desired target for the process variable.

`kp`/`ki`/`kd` can be changed during operation and can therefore be a function
of time.

If you're interested in the dependent form, add your own logic that computes
`kp`/`ki`/`kd` using dead time, time constant, `kc`, or whatever else.

## Todo

- [ ] Helper for (auto-)tuning by detecting frequency & amplitude of
      oscillations.

## License

Licensed under either at your discretion:

- Apache License, Version 2.0 (LICENSE-APACHE or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license (LICENSE-MIT or http://opensource.org/licenses/MIT)
