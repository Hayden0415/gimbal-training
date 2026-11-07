#![no_std]
#![no_main]
#![forbid(unsafe_code)]

mod dm_mc02;

use core::sync::atomic::{AtomicU32, Ordering};
use dm_mc02::Can1;
use embassy_executor::Spawner;
use embassy_stm32::can::Frame;
use embassy_time::{Duration, Instant, Ticker};

// This rig uses GM6020 voltage commands for motor IDs 1 to 4.
const MOTOR_COMMAND_ID: u16 = 0x1FF;
// Ticks without remote input before it counts as lost.
const REMOTE_SILENT_TICKS: u16 = 100;
const TICK: Duration = Duration::from_millis(1);

// Motor IDs on the teaching rig. Read the DIP switches (bit0..2) on each motor
// at the bench and update these if the wiring differs.
const YAW_MOTOR_ID: u8 = 1;
const PITCH_MOTOR_ID: u8 = 2;
// GM6020 feedback frames use 0x204 + motor ID (0x205..=0x20B).
const FEEDBACK_BASE_ID: u16 = 0x204;
// Final voltage clamp applied in `transmit`. Protocol hard limit is +/-25000;
// start smaller while tuning and only raise it once the loops are stable.
const VOLTAGE_LIMIT: f32 = 8000.0;
// A motor's feedback is considered "fresh" if received within this many ticks.
const FEEDBACK_FRESH_TICKS: u16 = 20;

// --- Starting PID gains (placeholders — tune on the bench with Tuning Studio) ---
// Position loop: rad in, rpm out. `maxout` caps the speed target.
const POS_KP: f32 = 10.0; // rpm per rad of error
const POS_KI: f32 = 0.0;
const POS_KD: f32 = 0.0;
const POS_MAXOUT_RPM: f32 = 150.0;
const POS_MAX_INTEGRAL: f32 = 100.0; // rpm * s
// Speed loop: rpm in, voltage counts out. `maxout` caps the voltage command.
const SPD_KP: f32 = 80.0; // counts per rpm of error
const SPD_KI: f32 = 0.0;
const SPD_KD: f32 = 0.0;
const SPD_MAX_INTEGRAL: f32 = 4000.0; // counts * s

// Control loop period, used by the PID as `dt_s`.
const DT_S: f32 = 0.001;

// Reference pose (calibration). These are the raw mechanical angle counts read
// when the rig is manually placed at "yaw forward, pitch level". Fill in at the bench.
const YAW_ZERO_COUNT: u16 = 0; // TODO: calibrate
const PITCH_ZERO_COUNT: u16 = 0; // TODO: calibrate

// Recenter (Prepare) tuning.
const RECENTER_ANGLE_EPS: f32 = 0.02; // rad; both axes must be within this of the pose
const RECENTER_SPEED_EPS: f32 = 5.0; // rpm; and slower than this
const RECENTER_SETTLE_TICKS: u16 = 300; // 0.3 s stable before Running
const RECENTER_TIMEOUT_TICKS: u16 = 3000; // 3 s to reach the pose

// Stick control (Running).
const STICK_DEADZONE: f32 = 0.02; // normalized stick input below this is treated as zero
const MAX_TARGET_SPEED_RAD_S: f32 = 1.5; // target-angle rate at full stick deflection
// Input direction: flip to -1.0 if an axis is installed the other way round.
const YAW_INPUT_DIR: f32 = 1.0;
const PITCH_INPUT_DIR: f32 = 1.0;
// Pitch target limits (rad). The rig's mechanical limits — calibrate at the bench.
const PITCH_MIN_RAD: f32 = -0.6;
const PITCH_MAX_RAD: f32 = 0.6;

// Fault detection (stall / high-frequency oscillation).
const STALL_TARGET_RPM: f32 = 20.0; // demanded speed must be at least this
const STALL_OUTPUT: f32 = 1500.0; // and output at least this many counts
const STALL_SPEED_RPM: f32 = 3.0; // while actual speed stays below this
const STALL_TICKS: u16 = 300; // 0.3 s of the above -> stall fault
const OSC_AMP_RPM: f32 = 20.0; // peak speed must exceed this to count one cycle
const OSC_FREQ_HZ: f32 = 8.0; // oscillation above this frequency is "high-frequency"
const OSC_TICKS: u16 = 200; // 0.2 s sustained -> oscillation fault

/// Command frames dropped because the CAN transmit queue was full, for example while no
/// motor acknowledges frames.
pub static TX_FAILED: AtomicU32 = AtomicU32::new(0);

/// Feedback and voltage command for one motor.
#[derive(Default)]
pub struct Motor {
    /// Single-turn motor shaft angle in rad; turns are not counted.
    pub angle_rad: f32,
    /// Raw single-turn mechanical angle as sent on the wire, 0..=8191 counts.
    /// Read this while holding the zero pose to calibrate `YAW_ZERO_COUNT`/`PITCH_ZERO_COUNT`.
    pub raw_count: u16,
    pub speed_rpm: f32,
    /// Feedback current as sent on the wire. The motor manual gives no scale to amperes.
    pub current_raw: i16,
    /// Voltage command counts, not volts; protocol range +/-25000.
    pub command_voltage_raw: f32,
}

impl Motor {
    /// Clear the voltage command in RAM. The motor receives zero when
    /// `Control::transmit` next runs, within 1 ms.
    pub fn disable(&mut self) {
        self.command_voltage_raw = 0.0;
    }

    pub fn read(&mut self, data: &[u8; 8]) {
        // GM6020 feedback, 0x204+ID, 8 bytes big-endian (manual "CAN 通信协议"):
        //   [0..2] mechanical angle, u16, 0..=8191
        //   [2..4] speed, i16, rpm
        //   [4..6] torque current, i16 (manual gives no scale to amperes)
        //   [6]    temperature, [7] unused
        let angle_raw = u16::from_be_bytes([data[0], data[1]]);
        let speed_raw = i16::from_be_bytes([data[2], data[3]]);
        let current_raw = i16::from_be_bytes([data[4], data[5]]);

        // Single-turn angle: 8192 counts per revolution -> radians.
        self.raw_count = angle_raw;
        self.angle_rad = angle_raw as f32 * (core::f32::consts::TAU / 8192.0);
        self.speed_rpm = speed_raw as f32;
        self.current_raw = current_raw;
    }
}

/// PID gains, plus the integral and history kept between ticks.
#[derive(Default)]
pub struct Pid {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    /// Accumulated integral (already multiplied by dt). Clamped to `[-max_integral, max_integral]`.
    pub integral: f32,
    /// Previous error, or `None` on the first call after `reset` (derivative is then zero,
    /// which avoids a derivative kick on the first sample).
    prev_error: Option<f32>,
    /// Symmetric output limit. The result is always within `[-maxout, maxout]`.
    pub maxout: f32,
    /// Symmetric anti-windup limit for the integral.
    pub max_integral: f32,
}

impl Pid {
    pub fn new(kp: f32, ki: f32, kd: f32, maxout: f32, max_integral: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            prev_error: None,
            maxout,
            max_integral,
        }
    }

    /// Clear the integral and error history.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = None;
    }

    /// `target` and `measured` share a unit, set by the caller. The position loop takes rad
    /// and returns rpm; the speed loop takes rpm and returns voltage counts. `dt_s` is in seconds.
    /// If a position loop returns rad/s, convert it to rpm before passing it to the speed loop.
    pub fn calculate(&mut self, target: f32, measured: f32, dt_s: f32) -> f32 {
        let error = target - measured;

        // Anti-windup: accumulate, then clamp the integral.
        self.integral = (self.integral + error * dt_s).clamp(-self.max_integral, self.max_integral);

        // Derivative on error; zero on the first sample after reset.
        let derivative = match self.prev_error {
            Some(prev) => (error - prev) / dt_s,
            None => 0.0,
        };
        self.prev_error = Some(error);

        let out = self.kp * error + self.ki * self.integral + self.kd * derivative;
        out.clamp(-self.maxout, self.maxout)
    }
}

/// The gimbal's stage, which decides the allowed operations and outputs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// Zero output. Waits for valid remote input and fresh feedback from both motors,
    /// then for a new enable request.
    Waiting,
    /// Recenters both axes to the reference pose with limited speed and output.
    /// Enters `Running` once both axes settle.
    Prepare,
    /// The left stick moves each axis's `target_angle_rad`, and its cascade tracks it.
    Running,
    /// Zero output until a manual recovery.
    Error,
}

/// Stall and high-frequency-oscillation detection state for one axis.
#[derive(Default)]
pub struct FaultDetector {
    /// Consecutive ticks the stall condition has held.
    pub stall_ticks: u16,
    /// Consecutive ticks the over-frequency oscillation has held.
    pub osc_ticks: u16,
    /// Latest oscillation frequency estimate in Hz.
    pub osc_freq_hz: f32,
    /// Ticks since the last qualifying speed sign change (half a cycle).
    half_ticks: u16,
    /// Peak |speed| since the last sign change, as an amplitude gate against noise.
    peak_speed: f32,
    /// Previous speed sample, for sign-change detection.
    last_speed: f32,
}

impl FaultDetector {
    fn reset(&mut self) {
        self.stall_ticks = 0;
        self.osc_ticks = 0;
        self.osc_freq_hz = 0.0;
        self.half_ticks = 0;
        self.peak_speed = 0.0;
        self.last_speed = 0.0;
    }

    fn sample(&mut self, speed_rpm: f32, target_rpm: f32, output: f32) {
        // Stall: motion is demanded with a large output, yet the speed stays near zero.
        // (A held position with zero target speed is NOT a stall.)
        if fabs(target_rpm) >= STALL_TARGET_RPM
            && fabs(output) >= STALL_OUTPUT
            && fabs(speed_rpm) <= STALL_SPEED_RPM
        {
            self.stall_ticks = self.stall_ticks.saturating_add(1);
        } else {
            self.stall_ticks = 0;
        }

        // Oscillation: estimate frequency from zero crossings of the speed signal.
        self.half_ticks = self.half_ticks.saturating_add(1);
        let abs_speed = fabs(speed_rpm);
        if abs_speed > self.peak_speed {
            self.peak_speed = abs_speed;
        }
        if self.last_speed * speed_rpm < 0.0 && self.peak_speed >= OSC_AMP_RPM {
            // A real half-cycle just completed; a full cycle is two sign changes.
            let period_s = 2.0 * self.half_ticks as f32 * DT_S;
            if period_s > 0.0 {
                self.osc_freq_hz = 1.0 / period_s;
            }
            self.half_ticks = 0;
            self.peak_speed = 0.0;
        }
        self.last_speed = speed_rpm;

        if self.osc_freq_hz >= OSC_FREQ_HZ {
            self.osc_ticks = self.osc_ticks.saturating_add(1);
        } else {
            self.osc_ticks = 0;
        }
    }
}

/// One gimbal axis: its motor, targets and cascaded PIDs.
#[derive(Default)]
pub struct Axis {
    pub motor: Motor,
    /// Position loop target in rad.
    pub target_angle_rad: f32,
    /// Speed loop target in rpm, set from the position loop output.
    pub target_rpm: f32,
    /// Position loop: rad in, rpm out.
    pub position_pid: Pid,
    /// Speed loop: rpm in, voltage counts out.
    pub speed_pid: Pid,
    /// Stall and oscillation detection state.
    pub fault: FaultDetector,
}

/// State shared by every control tick.
// TODO: add feedback ages and a fault record.
pub struct Control {
    pub state: State,
    pub yaw: Axis,
    pub pitch: Axis,
    pub remote: Option<dm_mc02::Dr16>,
    pub remote_silent_ticks: u16,
    /// Ticks since the last valid feedback frame for each axis (0 = just received).
    pub yaw_feedback_age: u16,
    pub pitch_feedback_age: u16,
    /// Right switch position on the previous tick, for detecting the enable gesture.
    pub prev_right_switch: Option<u8>,
    /// Left switch position on the previous tick, for detecting the recovery gesture.
    pub prev_left_switch: Option<u8>,
    /// Ticks spent in Prepare (for the recenter timeout).
    pub recenter_ticks: u16,
    /// Consecutive ticks both axes have been settled (for entering Running).
    pub recenter_stable_ticks: u16,
}

/// Start in `Waiting` with no remote input.
/// Set the initial value of each field you add here, including the PID gains.
impl Default for Control {
    fn default() -> Self {
        Self {
            state: State::Waiting,
            yaw: Axis {
                position_pid: Pid::new(POS_KP, POS_KI, POS_KD, POS_MAXOUT_RPM, POS_MAX_INTEGRAL),
                speed_pid: Pid::new(SPD_KP, SPD_KI, SPD_KD, VOLTAGE_LIMIT, SPD_MAX_INTEGRAL),
                ..Default::default()
            },
            pitch: Axis {
                position_pid: Pid::new(POS_KP, POS_KI, POS_KD, POS_MAXOUT_RPM, POS_MAX_INTEGRAL),
                speed_pid: Pid::new(SPD_KP, SPD_KI, SPD_KD, VOLTAGE_LIMIT, SPD_MAX_INTEGRAL),
                ..Default::default()
            },
            remote: None,
            remote_silent_ticks: REMOTE_SILENT_TICKS,
            // Start stale: no feedback has arrived yet.
            yaw_feedback_age: FEEDBACK_FRESH_TICKS,
            pitch_feedback_age: FEEDBACK_FRESH_TICKS,
            prev_right_switch: None,
            prev_left_switch: None,
            recenter_ticks: 0,
            recenter_stable_ticks: 0,
        }
    }
}

/// Encode one motor's voltage command as a signed i16 big-endian into the byte
/// slot for its motor ID in a `0x1FF` group command (IDs 1..=4 -> slots 0,2,4,6).
fn encode_voltage(data: &mut [u8; 8], motor_id: u8, value: f32) {
    let slot = (motor_id - 1) as usize * 2;
    // no_std `f32` has no `round`; round half away from zero manually.
    let rounded = if value >= 0.0 { value + 0.5 } else { value - 0.5 };
    let encoded = rounded as i16;
    data[slot..slot + 2].copy_from_slice(&encoded.to_be_bytes());
}

/// Convert a raw GM6020 mechanical angle count (0..=8191) to radians.
fn count_to_rad(count: u16) -> f32 {
    count as f32 * (core::f32::consts::TAU / 8192.0)
}

/// Wrap an angle to [-PI, PI) so the position loop takes the shortest path
/// across the 0/2PI boundary instead of seeing a huge jump.
fn wrap_pi(mut a: f32) -> f32 {
    while a > core::f32::consts::PI {
        a -= core::f32::consts::TAU;
    }
    while a < -core::f32::consts::PI {
        a += core::f32::consts::TAU;
    }
    a
}

/// Run one axis's cascade: position loop (rad -> rpm), then speed loop (rpm -> voltage).
fn update_axis(axis: &mut Axis) {
    let measured = axis.motor.angle_rad;
    // Wrap the angle error so the loop takes the shortest path.
    let error = wrap_pi(axis.target_angle_rad - measured);
    let wrapped_target = measured + error;
    axis.target_rpm = axis.position_pid.calculate(wrapped_target, measured, DT_S);
    axis.motor.command_voltage_raw =
        axis.speed_pid.calculate(axis.target_rpm, axis.motor.speed_rpm, DT_S);
}

/// True when one axis is close to its target and moving slowly enough to count as settled.
fn axis_settled(axis: &Axis) -> bool {
    let err = wrap_pi(axis.target_angle_rad - axis.motor.angle_rad);
    err >= -RECENTER_ANGLE_EPS
        && err <= RECENTER_ANGLE_EPS
        && axis.motor.speed_rpm >= -RECENTER_SPEED_EPS
        && axis.motor.speed_rpm <= RECENTER_SPEED_EPS
}

/// Zero the stick input inside the deadzone so a resting stick produces no drift.
fn deadzone(v: f32, dz: f32) -> f32 {
    if v > dz || v < -dz {
        v
    } else {
        0.0
    }
}

/// Absolute value for f32 (no_std core has no inherent `abs` method to rely on here).
fn fabs(v: f32) -> f32 {
    if v < 0.0 {
        -v
    } else {
        v
    }
}

impl Control {
    /// Clear both motors' voltage commands in RAM.
    pub fn disable_all(&mut self) {
        self.yaw.motor.disable();
        self.pitch.motor.disable();
    }

    /// Both axes have received feedback within the freshness window.
    fn feedback_fresh(&self) -> bool {
        self.yaw_feedback_age < FEEDBACK_FRESH_TICKS && self.pitch_feedback_age < FEEDBACK_FRESH_TICKS
    }

    /// The right switch has just moved from down (2) to middle (3): the "enable" gesture.
    fn just_enabled(&self) -> bool {
        matches!(self.prev_right_switch, Some(2))
            && matches!(self.remote.map(|r| r.right_switch), Some(3))
    }

    /// Enter `Prepare`: aim both axes at the calibrated reference pose and reset the PIDs.
    fn enter_prepare(&mut self) {
        self.state = State::Prepare;
        self.yaw.target_angle_rad = count_to_rad(YAW_ZERO_COUNT);
        self.pitch.target_angle_rad = count_to_rad(PITCH_ZERO_COUNT);
        self.yaw.position_pid.reset();
        self.yaw.speed_pid.reset();
        self.pitch.position_pid.reset();
        self.pitch.speed_pid.reset();
        self.recenter_ticks = 0;
        self.recenter_stable_ticks = 0;
        self.reset_faults();
    }

    /// The right switch is currently at down (2): the "disable" request.
    fn disable_requested(&self) -> bool {
        matches!(self.remote.map(|r| r.right_switch), Some(2))
    }

    /// Manual recovery gesture: right switch held at middle (3) while the left switch
    /// moves from down (2) to up (1).
    fn recovery_requested(&self) -> bool {
        matches!(self.remote.map(|r| r.right_switch), Some(3))
            && matches!(self.prev_left_switch, Some(2))
            && matches!(self.remote.map(|r| r.left_switch), Some(1))
    }

    /// Leave Error and resume from the current pose, without recentering again.
    fn enter_running_from_error(&mut self) {
        self.state = State::Running;
        self.yaw.target_angle_rad = self.yaw.motor.angle_rad;
        self.pitch.target_angle_rad = self.pitch.motor.angle_rad;
        self.yaw.target_rpm = 0.0;
        self.pitch.target_rpm = 0.0;
        self.yaw.position_pid.reset();
        self.yaw.speed_pid.reset();
        self.pitch.position_pid.reset();
        self.pitch.speed_pid.reset();
        self.reset_faults();
    }

    /// Clear both axes' fault-detection state.
    fn reset_faults(&mut self) {
        self.yaw.fault.reset();
        self.pitch.fault.reset();
    }

    /// Enter Error: zero both outputs and log which axis faulted and why.
    fn enter_error(&mut self, axis: &str, reason: &str) {
        self.state = State::Error;
        self.disable_all();
        rtt_target::rprintln!("FAULT: {} {}", axis, reason);
    }

    /// Sample both axes and trigger Error if a stall or sustained oscillation is detected.
    /// Runs after `update` so it sees this tick's fresh targets and outputs.
    fn check_faults(&mut self) {
        if !matches!(self.state, State::Prepare | State::Running) {
            return;
        }

        let yaw_speed = self.yaw.motor.speed_rpm;
        let yaw_target = self.yaw.target_rpm;
        let yaw_out = self.yaw.motor.command_voltage_raw;
        self.yaw.fault.sample(yaw_speed, yaw_target, yaw_out);

        let pitch_speed = self.pitch.motor.speed_rpm;
        let pitch_target = self.pitch.target_rpm;
        let pitch_out = self.pitch.motor.command_voltage_raw;
        self.pitch.fault.sample(pitch_speed, pitch_target, pitch_out);

        if self.yaw.fault.stall_ticks >= STALL_TICKS {
            self.enter_error("yaw", "stall");
        } else if self.yaw.fault.osc_ticks >= OSC_TICKS {
            self.enter_error("yaw", "oscillation");
        } else if self.pitch.fault.stall_ticks >= STALL_TICKS {
            self.enter_error("pitch", "stall");
        } else if self.pitch.fault.osc_ticks >= OSC_TICKS {
            self.enter_error("pitch", "oscillation");
        }
    }

    /// Runs every 1 ms: read the remote and motor feedback, update the state, then
    /// compute and send the outputs. Missing remote input stops output; a latched Error is preserved.
    pub fn tick(&mut self, can: &mut Can1) {
        self.read_remote();
        self.read_feedback(can);
        self.update_mode();
        if self.remote.is_none() && self.state != State::Error {
            self.state = State::Waiting;
        }
        self.update();
        self.check_faults();
        self.transmit(can);
    }

    /// Take the latest input published by the remote task. After an invalid frame, or
    /// `REMOTE_SILENT_TICKS` ticks without one, clear the commands; return to Waiting unless already in Error.
    pub fn read_remote(&mut self) {
        self.remote_silent_ticks = self.remote_silent_ticks.saturating_add(1);
        if let Some(remote) = dm_mc02::REMOTE.try_take() {
            self.remote = remote;
            if self.remote.is_some() {
                self.remote_silent_ticks = 0;
            }
        }
        if self.remote_silent_ticks >= REMOTE_SILENT_TICKS {
            self.remote = None;
        }
        if self.remote.is_none() {
            self.disable_all();
            if self.state != State::Error {
                self.state = State::Waiting;
            }
        }
    }

    /// Read every queued CAN frame without waiting for more.
    pub fn read_feedback(&mut self, can: &mut Can1) {
        // Age grows every tick; a valid frame resets its axis back to 0.
        self.yaw_feedback_age = self.yaw_feedback_age.saturating_add(1);
        self.pitch_feedback_age = self.pitch_feedback_age.saturating_add(1);

        let receiver = can.reader();
        while let Ok(received) = receiver.try_receive() {
            match received {
                Ok(envelope) => {
                    let frame = envelope.frame;
                    // Only classic standard-ID frames with exactly 8 bytes are feedback.
                    let embedded_can::Id::Standard(id) = frame.id() else {
                        continue;
                    };
                    let data: &[u8; 8] = match frame.data().try_into() {
                        Ok(d) => d,
                        Err(_) => continue,
                    };
                    match id.as_raw() {
                        id if id == FEEDBACK_BASE_ID + YAW_MOTOR_ID as u16 => {
                            self.yaw.motor.read(data);
                            self.yaw_feedback_age = 0;
                        }
                        id if id == FEEDBACK_BASE_ID + PITCH_MOTOR_ID as u16 => {
                            self.pitch.motor.read(data);
                            self.pitch_feedback_age = 0;
                        }
                        _ => {}
                    }
                }
                Err(error) => dm_mc02::log_can_error(error),
            }
        }
    }

    /// Choose the state from the remote, feedback and fault conditions.
    pub fn update_mode(&mut self) {
        // Snapshot this tick's switch positions, then compare against the previous ones.
        let right_switch = self.remote.map(|r| r.right_switch);
        let left_switch = self.remote.map(|r| r.left_switch);

        match self.state {
            State::Waiting => {
                // Start only on fresh feedback AND a new enable gesture (down -> middle).
                if self.feedback_fresh() && self.just_enabled() {
                    self.enter_prepare();
                }
            }
            State::Prepare => {
                self.recenter_ticks = self.recenter_ticks.saturating_add(1);

                // Count consecutive settled ticks; any movement resets the counter.
                if axis_settled(&self.yaw) && axis_settled(&self.pitch) {
                    self.recenter_stable_ticks = self.recenter_stable_ticks.saturating_add(1);
                } else {
                    self.recenter_stable_ticks = 0;
                }

                if self.recenter_stable_ticks >= RECENTER_SETTLE_TICKS {
                    self.state = State::Running;
                } else if self.recenter_ticks >= RECENTER_TIMEOUT_TICKS {
                    // Recenter timed out: stop output and go back to Waiting.
                    self.disable_all();
                    self.state = State::Waiting;
                }
            }
            State::Running => {
                if self.disable_requested() {
                    self.disable_all();
                    self.state = State::Waiting;
                } else if let Some(remote) = self.remote {
                    let yaw_input =
                        YAW_INPUT_DIR * deadzone(remote.channels[2] as f32 / 660.0, STICK_DEADZONE);
                    let pitch_input =
                        PITCH_INPUT_DIR * deadzone(remote.channels[3] as f32 / 660.0, STICK_DEADZONE);
                    // The stick sets the target's *rate of change*, not the angle itself.
                    self.yaw.target_angle_rad += yaw_input * MAX_TARGET_SPEED_RAD_S * DT_S;
                    self.pitch.target_angle_rad =
                        (self.pitch.target_angle_rad + pitch_input * MAX_TARGET_SPEED_RAD_S * DT_S)
                            .clamp(PITCH_MIN_RAD, PITCH_MAX_RAD);
                }
            }
            State::Error => {
                if self.recovery_requested() {
                    self.enter_running_from_error();
                }
            }
        }

        self.prev_right_switch = right_switch;
        self.prev_left_switch = left_switch;
    }

    /// Set targets for the current state, run each axis's PIDs and write the voltage commands.
    pub fn update(&mut self) {
        match self.state {
            State::Waiting | State::Error => self.disable_all(),
            State::Prepare | State::Running => {
                update_axis(&mut self.yaw);
                update_axis(&mut self.pitch);
            }
        }
    }

    /// Queue one group command frame on CAN1 without waiting. This runs every tick, so a
    /// cleared command reaches the motor within 1 ms. A full queue drops the frame and
    /// counts it in `TX_FAILED`.
    pub fn transmit(&mut self, can: &mut Can1) {
        let mut data = [0u8; 8];

        // Clamp each output to the bench limit, then encode into its motor-ID slot.
        // Zero (our idle/disabled value) encodes to two zero bytes, so a disabled
        // motor receives a genuine zero-voltage command.
        let yaw_out = self.yaw.motor.command_voltage_raw.clamp(-VOLTAGE_LIMIT, VOLTAGE_LIMIT);
        let pitch_out = self
            .pitch
            .motor
            .command_voltage_raw
            .clamp(-VOLTAGE_LIMIT, VOLTAGE_LIMIT);
        encode_voltage(&mut data, YAW_MOTOR_ID, yaw_out);
        encode_voltage(&mut data, PITCH_MOTOR_ID, pitch_out);

        let frame = Frame::new_standard(MOTOR_COMMAND_ID, &data).unwrap();
        if can.writer().try_write(frame).is_err() {
            TX_FAILED.fetch_add(1, Ordering::Relaxed);
        }
    }
}

// Set up RTT and the board, then start the remote and control tasks.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    rtt_target::rtt_init_print!(rtt_target::ChannelMode::NoBlockTrim, 1024);
    let (can, watchdog, remote) = dm_mc02::init();
    rtt_target::rprintln!("Gimbal control started");
    spawner.spawn(dm_mc02::remote_task(remote).unwrap());
    spawner.spawn(control_task(can, watchdog).unwrap());
}

// Run `tick` at 1 kHz and feed the watchdog after each one. If a tick hangs, the watchdog
// resets the MCU after 100 ms.
#[embassy_executor::task]
async fn control_task(mut can: Can1, mut watchdog: dm_mc02::Watchdog) {
    let mut control = Control::default();
    let mut ticker = Ticker::every(TICK);
    let mut last = Instant::now();
    let mut log_ticks = 0u16;
    loop {
        ticker.next().await;
        // After a stall, such as the 20 ms halt when a debug probe attaches, start the
        // schedule again instead of running every missed tick back to back.
        let now = Instant::now();
        if now - last > TICK * 2 {
            ticker.reset();
        }
        last = now;
        control.tick(&mut can);

        // Rate-limited RTT log (~every 200 ms) for the values we care about.
        log_ticks += 1;
        if log_ticks >= 200 {
            log_ticks = 0;
            rtt_target::rprintln!(
                "state={:?} | yaw: ang={:.3} tgt={:.3} rpm={:.1} trpm={:.1} | pitch: ang={:.3} tgt={:.3} | out y={:.0} p={:.0} | osc y={:.1} p={:.1} | raw y={} p={}",
                control.state,
                control.yaw.motor.angle_rad,
                control.yaw.target_angle_rad,
                control.yaw.motor.speed_rpm,
                control.yaw.target_rpm,
                control.pitch.motor.angle_rad,
                control.pitch.target_angle_rad,
                control.yaw.motor.command_voltage_raw,
                control.pitch.motor.command_voltage_raw,
                control.yaw.fault.osc_freq_hz,
                control.pitch.fault.osc_freq_hz,
                control.yaw.motor.raw_count,
                control.pitch.motor.raw_count,
            );
        }

        watchdog.pet();
    }
}
