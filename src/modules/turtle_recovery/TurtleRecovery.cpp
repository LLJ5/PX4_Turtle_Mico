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
//主程序：读取遥控、姿态、角速度，检查启动条件，发送恢复请求

#include "TurtleObserver.hpp"
#include <turtle_recovery/TurtleFlipController.hpp>

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <parameters/param.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/turtle_recovery_control.h>
#include <uORB/topics/turtle_recovery_feedback.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class TurtleRecovery : public ModuleBase<TurtleRecovery>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	TurtleRecovery() : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
	{
		updateGeometry();
	}

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

private:
	void Run() override;
	void updateGeometry();
	void updateRecovery(hrt_abstime now);
	void publishRecovery(const turtle_recovery::Request &request);
	float auxInput() const;
	static bool fresh(hrt_abstime now, hrt_abstime timestamp, hrt_abstime timeout)
	{
		return timestamp != 0 && now >= timestamp && now - timestamp <= timeout;
	}

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _manual_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _land_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _feedback_sub{ORB_ID(turtle_recovery_feedback)};
	uORB::Publication<turtle_recovery_control_s> _recovery_pub{ORB_ID(turtle_recovery_control)};
	actuator_armed_s _armed{};
	turtle_recovery_feedback_s _feedback{};
	turtle_recovery::TurtleFlipController _controller;
	bool _cancel_recovery{false};
	bool _recovery_safe{false};
	bool _recovery_entry{false};
	bool _feedback_fresh{false};
	uint8_t _recovery_quat_reset{0};

	manual_control_setpoint_s _manual{};
	vehicle_attitude_s _attitude{};
	vehicle_angular_velocity_s _angular_velocity{};
	vehicle_status_s _vehicle_status{};
	vehicle_land_detected_s _land{};
	turtle_recovery::ObserveTrigger _trigger;
	turtle_recovery::SwitchPosition _switch_position{turtle_recovery::SwitchPosition::Unknown};
	bool _observing{false};

	uint32_t _right_motor_mask{0};
	unsigned _trigger_count{0};
	float _position_y[4] {};
	float _aux{NAN};
	float _alignment{NAN};
	float _tilt_deg{NAN};
	float _roll_rate_deg_s{NAN};
	bool _geometry_valid{false};
	bool _rc_valid{false};
	bool _attitude_valid{false};
	bool _gyro_valid{false};
	bool _status_valid{false};
	bool _land_valid{false};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::TRTL_OBS_EN>) _param_obs_en,
		(ParamInt<px4::params::TRTL_AUX>) _param_aux,
		(ParamFloat<px4::params::COM_RC_LOSS_T>) _param_rc_loss_t,
		(ParamBool<px4::params::TRTL_ENABLE>) _param_enable,
		(ParamFloat<px4::params::TRTL_MAX_THR>) _param_max_thr,
		(ParamFloat<px4::params::TRTL_R_RATE>) _param_rate,
		(ParamFloat<px4::params::TRTL_REL_ANG>) _param_release_angle,
		(ParamFloat<px4::params::TRTL_NP_T>) _param_no_progress_time,
		(ParamFloat<px4::params::TRTL_DRV_T>) _param_drive_time,
		(ParamFloat<px4::params::TRTL_THR_SLEW>) _param_throttle_slew
	)
};

void TurtleRecovery::updateGeometry()
{
	_geometry_valid = false;
	_right_motor_mask = 0;
	int32_t rotor_count = 0;

	if (param_get(param_find("CA_ROTOR_COUNT"), &rotor_count) != PX4_OK || rotor_count != 4) {
		return;
	}

	for (unsigned i = 0; i < 4; ++i) {
		char name[17];
		snprintf(name, sizeof(name), "CA_ROTOR%u_PY", i);

		if (param_get(param_find(name), &_position_y[i]) != PX4_OK) {
			return;
		}
	}

	_geometry_valid = turtle_recovery::rightMotorMask(_position_y, _right_motor_mask);
}

float TurtleRecovery::auxInput() const
{
	switch (_param_aux.get()) {
	case 1: return _manual.aux1;

	case 2: return _manual.aux2;

	case 3: return _manual.aux3;

	case 4: return _manual.aux4;

	case 5: return _manual.aux5;

	case 6: return _manual.aux6;

	default: return NAN;
	}
}

void TurtleRecovery::Run()
{
	if (should_exit()) {
		auto stop = _controller.request(hrt_absolute_time());
		stop.action = turtle_recovery::Action::Stop;
		stop.throttle = 0.f;
		publishRecovery(stop);
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		_cancel_recovery |= _controller.active();
		parameter_update_s update;
		_parameter_update_sub.copy(&update);
		const bool previous_enable = _param_obs_en.get();
		const int32_t previous_aux = _param_aux.get();
		updateParams();
		updateGeometry();

		if (previous_enable != _param_obs_en.get() || previous_aux != _param_aux.get()) {
			_trigger.reset();
		}
	}

	_manual_sub.update(&_manual);
	_attitude_sub.update(&_attitude);
	_angular_velocity_sub.update(&_angular_velocity);
	_vehicle_status_sub.update(&_vehicle_status);
	_land_sub.update(&_land);
	_armed_sub.update(&_armed);
	_feedback_sub.update(&_feedback);

	const hrt_abstime now = hrt_absolute_time();
	_aux = auxInput();
	_rc_valid = _manual.valid && _manual.data_source == manual_control_setpoint_s::SOURCE_RC
		    && turtle_recovery::rcSampleFresh(now, _manual.timestamp_sample, _param_rc_loss_t.get())
		    && PX4_ISFINITE(_aux) && fabsf(_aux) <= 1.f;
	_switch_position = turtle_recovery::switchPosition(_rc_valid, _aux);
	_status_valid = fresh(now, _vehicle_status.timestamp, 2_s);
	_land_valid = fresh(now, _land.timestamp, 2_s);
	_attitude_valid = fresh(now, _attitude.timestamp_sample, 100_ms)
			  && turtle_recovery::attitudeAlignment(_attitude.q, _alignment);
	_tilt_deg = _attitude_valid ? math::degrees(acosf(math::constrain(_alignment, -1.f, 1.f))) : NAN;

	if (!_attitude_valid) {
		_alignment = NAN;
	}

	_gyro_valid = fresh(now, _angular_velocity.timestamp_sample, 100_ms)
		      && matrix::Vector3f{_angular_velocity.xyz}.isAllFinite();
	_roll_rate_deg_s = _gyro_valid ? math::degrees(_angular_velocity.xyz[0]) : NAN;

	const bool switch_available = _param_obs_en.get() && _rc_valid && _status_valid
				      && _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_DISARMED;
	_observing = switch_available && _switch_position == turtle_recovery::SwitchPosition::High;

	if (_trigger.update(switch_available, _aux)) {
		++_trigger_count;
		PX4_INFO("RC request #%u: tilt %.1f deg, right mask 0x%x",
			 _trigger_count, (double)_tilt_deg, (unsigned)_right_motor_mask);
	}

	updateRecovery(now);
}

void TurtleRecovery::publishRecovery(const turtle_recovery::Request &request)
{
	turtle_recovery_control_s message{};
	message.timestamp = request.timestamp;
	message.session_id = request.session;
	message.action = static_cast<uint8_t>(request.action);
	message.throttle = request.throttle;
	_recovery_pub.publish(message);
}

void TurtleRecovery::updateRecovery(hrt_abstime now)
{
	using Controller = turtle_recovery::TurtleFlipController;
	Controller::Settings settings{};
	settings.max_throttle = _param_max_thr.get();
	settings.target_rate = _param_rate.get();
	settings.release_tilt = _param_release_angle.get();
	settings.no_progress_timeout = _param_no_progress_time.get();
	settings.drive_timeout = _param_drive_time.get();
	settings.throttle_slew = _param_throttle_slew.get();
	Controller::Input in{};
	in.now = now;
	in.enabled = _param_enable.get();
	in.low = _rc_valid && _switch_position == turtle_recovery::SwitchPosition::Low;
	in.high = _rc_valid && _switch_position == turtle_recovery::SwitchPosition::High;
	const bool reset_unchanged = !_controller.active() || _recovery_quat_reset == _attitude.quat_reset_counter;
	const bool settings_valid = PX4_ISFINITE(_param_max_thr.get()) && _param_max_thr.get() >= 0.05f
				    && _param_max_thr.get() <= turtle_recovery::HARD_MAX_THROTTLE
				    && PX4_ISFINITE(_param_rate.get()) && _param_rate.get() >= 30.f
				    && _param_rate.get() <= 200.f && PX4_ISFINITE(_param_release_angle.get())
				    && _param_release_angle.get() >= 30.f && _param_release_angle.get() <= 90.f
				    && Controller::validTiming(settings) && Controller::validSlew(settings);
	in.safe = _rc_valid && _status_valid && _attitude_valid && _gyro_valid && fresh(now, _armed.timestamp, 2_s)
		  && !_armed.armed && !_armed.kill && !_armed.lockdown && !_armed.termination && !_armed.in_esc_calibration_mode
		  && PX4_ISFINITE(_manual.throttle) && _manual.throttle < -0.8f && reset_unchanged && !_cancel_recovery
		  && (!_vehicle_status.safety_button_available || _vehicle_status.safety_off)
		  && _vehicle_status.power_input_valid && !_vehicle_status.calibration_enabled && !_vehicle_status.rc_calibration_in_progress
		  && !_vehicle_status.failsafe && (_vehicle_status.failure_detector_status & ~3u) == 0 && settings_valid;
	_cancel_recovery = false;
	const matrix::Dcmf rotation{matrix::Quatf{_attitude.q}};
	in.tilt = _tilt_deg;
	in.phase = atan2f(rotation(2, 1), rotation(2, 2));
	in.roll_rate = _roll_rate_deg_s;
	in.gyro_norm = math::degrees(matrix::Vector3f{_angular_velocity.xyz}.norm());
	in.safe &= PX4_ISFINITE(in.gyro_norm) && in.gyro_norm < 400.f && fabsf(rotation(2, 0)) < 0.5f;
	in.entry = in.safe && _geometry_valid && _right_motor_mask == turtle_recovery::RIGHT_MOTORS
		   && _land_valid && _land.landed && _tilt_deg > 150.f && in.gyro_norm < 10.f && fabsf(rotation(2, 0)) < 0.35f;
	in.feedback_fresh = fresh(now, _feedback.timestamp, 200_ms);
	in.backend_idle = _feedback.state == turtle_recovery_feedback_s::STATE_IDLE && !_feedback.busy;
	in.backend_clean = !_feedback.direction_dirty;
	in.backend_ready = _feedback.state == turtle_recovery_feedback_s::STATE_READY
			   || _feedback.state == turtle_recovery_feedback_s::STATE_DRIVING;
	in.backend_session = _feedback.session_id;
	_recovery_safe = in.safe;
	_recovery_entry = in.entry;
	_feedback_fresh = in.feedback_fresh;
	const auto previous_state = _controller.state();
	const auto request = _controller.update(in, settings);

	if (previous_state != _controller.state()) {
		PX4_INFO("Recovery state %u: %s", static_cast<unsigned>(_controller.state()), _controller.reason());
	}

	if (!_controller.active()) { _recovery_quat_reset = _attitude.quat_reset_counter; }

	publishRecovery(request);
}

int TurtleRecovery::print_status()
{
	PX4_INFO("Powered recovery: %s, state %u: %s", _param_enable.get() ? "ENABLED" : "disabled",
		 static_cast<unsigned>(_controller.state()), _controller.reason());
	PX4_INFO("Recovery progress %.1f deg, throttle %.3f (limit %.3f), backend state %u, busy %s, direction dirty %s",
		 (double)_controller.progress(), (double)_controller.throttle(), (double)_param_max_thr.get(),
		 _feedback.state, _feedback.busy ? "yes" : "no", _feedback.direction_dirty ? "yes" : "no");
	PX4_INFO("Recovery checks: safe %s, inverted rest %s, feedback fresh %s",
		 _recovery_safe ? "yes" : "no", _recovery_entry ? "yes" : "no", _feedback_fresh ? "yes" : "no");
	PX4_INFO("Drive time limits: no progress %.1f s, total %.1f s (DShot hard limit 3.0 s)",
		 (double)_param_no_progress_time.get(), (double)_param_drive_time.get());
	PX4_INFO("Throttle slew: %.2f /s (zero roll rate), hard throttle limit %.2f",
		 (double)_param_throttle_slew.get(), (double)turtle_recovery::HARD_MAX_THROTTLE);
	PX4_INFO("Observation: %s, input: AUX%" PRId32, _param_obs_en.get() ? "enabled" : "disabled", _param_aux.get());
	const char *position = "UNKNOWN (invalid/stale)";

	switch (_switch_position) {
	case turtle_recovery::SwitchPosition::High: position = "HIGH (+1)"; break;

	case turtle_recovery::SwitchPosition::Low: position = "LOW (-1)"; break;

	case turtle_recovery::SwitchPosition::Middle: position = "MIDDLE"; break;

	case turtle_recovery::SwitchPosition::Unknown: break;
	}

	PX4_INFO("Switch: %s", position);
	PX4_INFO("RC: %s, AUX: %.2f, OFF seen: %s, observing: %s, requests: %u",
		 _rc_valid ? "valid" : "invalid/stale", (double)_aux, _trigger.seenOff() ? "yes" : "no",
		 _observing ? "yes" : "no", _trigger_count);
	PX4_INFO("Vehicle: %s", !_status_valid ? "unknown/stale" :
		 (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED ? "armed" : "disarmed"));
	PX4_INFO("Attitude: %s, tilt: %.1f deg, body-down alignment: %.3f", _attitude_valid ? "valid" : "invalid/stale",
		 (double)_tilt_deg, (double)_alignment);
	PX4_INFO("Gyro: %s, roll rate: %.1f deg/s (+ = body-right roll)", _gyro_valid ? "valid" : "invalid/stale",
		 (double)_roll_rate_deg_s);
	PX4_INFO("Land detector (recovery entry gate): %s", !_land_valid ? "unknown/stale" :
		 (_land.landed ? "landed" : "not landed"));
	PX4_INFO("Geometry: %s, right logical motor mask: 0x%x", _geometry_valid ? "supported" : "unsupported",
		 (unsigned)_right_motor_mask);

	if (_geometry_valid) {
		for (unsigned i = 0; i < 4; ++i) {
			PX4_INFO("Motor %u: y=%.3f, %s", i + 1, (double)_position_y[i],
				 (_right_motor_mask & (1u << i)) ? "RIGHT (recovery motor)" : "LEFT");
		}
	}

	return PX4_OK;
}

int TurtleRecovery::task_spawn(int argc, char *argv[])
{
	TurtleRecovery *instance = new TurtleRecovery();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;
		instance->ScheduleOnInterval(20_ms);
		return PX4_OK;
	}

	PX4_ERR("alloc failed");
	return PX4_ERROR;
}

int TurtleRecovery::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description
Observe RC input, quaternion attitude, roll rate and right-side motor geometry.
TRTL_ENABLE separately enables powered disarmed recovery using Motor 1 and 4.
Requires LOW then HIGH, low throttle, inverted rest, valid sensors and landed.
Recovery stops on switch OFF, RC loss, invalid sensors, wrong direction or timeout.
The DShot backend independently stops and restores direction on lease expiration.
Enable TRTL_OBS_EN and map a spare RC switch to the AUX input selected by TRTL_AUX.
Switch position is displayed directly. Observation follows HIGH while enabled
and disarmed. Requests count OFF-to-ON transitions separately; starting with ON
or recovering from RC loss requires OFF before another request is counted.
Remove all propellers for initial tests. Starts at boot on MicoAir H743 V2 when
TRTL_ENABLE is set. Normal flight arming is inhibited until direction is restored.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("turtle_recovery", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return PX4_OK;
}

extern "C" __EXPORT int turtle_recovery_main(int argc, char *argv[])
{
	return TurtleRecovery::main(argc, argv);
}
