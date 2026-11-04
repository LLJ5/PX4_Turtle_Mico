/****************************************************************************
 *
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
//定义 TRTL_OBS_EN、TRTL_AUX   定义油门、爬升率、收油角、时限等参数
/**
 * Enable turtle recovery observation
 *
 * Enables RC switch observation only. The turtle_recovery module must be
 * started manually. Motor output is separately controlled by TRTL_ENABLE.
 *
 * @boolean
 * @group Turtle Recovery
 */
PARAM_DEFINE_INT32(TRTL_OBS_EN, 0);

/**
 * Turtle recovery AUX input
 *
 * Selects manual_control_setpoint AUX1 through AUX6. Map a spare RC channel
 * using the corresponding RC_MAP_AUX parameter. Powered recovery requires
 * a valid RC source, a disarmed vehicle, and an OFF to ON switch transition.
 *
 * @min 1
 * @max 6
 * @group Turtle Recovery
 */
PARAM_DEFINE_INT32(TRTL_AUX, 1);

/**
 * Enable powered automatic right-side turtle recovery
 *
 * Allows disarmed Motor 1 and Motor 4 to reverse and run briefly when inverted.
 * Requires RC LOW followed by HIGH, low throttle, valid attitude, landed and rest.
 * Starts turtle_recovery at boot on MicoAir H743 V2. Remove props for first tests.
 *
 * @boolean
 * @group Turtle Recovery
 */
PARAM_DEFINE_INT32(TRTL_ENABLE, 0);

/**
 * Maximum recovery DShot throttle fraction
 *
 * This is a protocol throttle fraction, not physical power or thrust percent.
 * Recovery duration is bounded by TRTL_DRV_T and TRTL_NP_T.
 *
 * @min 0.05
 * @max 0.8
 * @decimal 2
 * @increment 0.01
 * @group Turtle Recovery
 */
PARAM_DEFINE_FLOAT(TRTL_MAX_THR, 0.2f);

/**
 * Target positive roll rate during recovery
 *
 * @min 30
 * @max 200
 * @unit deg/s
 * @group Turtle Recovery
 */
PARAM_DEFINE_FLOAT(TRTL_R_RATE, 120.f);

/**
 * Remaining tilt at recovery thrust release
 *
 * Stop thrust before upright and let inertia and ground contact finish the roll.
 * A short rate-based prediction may release thrust earlier.
 *
 * @min 30
 * @max 90
 * @unit deg
 * @group Turtle Recovery
 */
PARAM_DEFINE_FLOAT(TRTL_REL_ANG, 75.f);

/**
 * Recovery no-progress timeout
 *
 * Stops recovery if positive roll progress is below 5 degrees after this time.
 * Measured from drive entry, including throttle ramp. Must not exceed TRTL_DRV_T.
 * Change only while idle. Switch OFF and other stop conditions still take priority.
 *
 * @min 0.5
 * @max 2.5
 * @unit s
 * @decimal 1
 * @increment 0.1
 * @group Turtle Recovery
 */
PARAM_DEFINE_FLOAT(TRTL_NP_T, 1.2f);

/**
 * Recovery total drive timeout
 *
 * Maximum duration of the drive phase, including throttle ramp.
 * Must be at least TRTL_NP_T. Does not extend the independent DShot 3 second limit.
 * Change only while idle. Reaching the release angle or other stop conditions ends drive sooner.
 *
 * @min 0.5
 * @max 2.8
 * @unit s
 * @decimal 1
 * @increment 0.1
 * @group Turtle Recovery
 */
PARAM_DEFINE_FLOAT(TRTL_DRV_T, 2.5f);

/**
 * Recovery throttle increase slew rate
 *
 * Normalized DShot throttle increase per second at zero roll rate.
 * For example 0.5 takes about 1 second to rise from zero to 0.5 while stationary.
 * Positive roll reduces the increase; above TRTL_R_RATE throttle decreases.
 * The requested throttle may not be reached before a stop condition or timeout.
 * Change only while idle. Does not change the recovery timeouts or ESC direction waits.
 *
 * @min 0.1
 * @max 1.0
 * @unit 1/s
 * @decimal 2
 * @increment 0.05
 * @group Turtle Recovery
 */
PARAM_DEFINE_FLOAT(TRTL_THR_SLEW, 0.25f);
