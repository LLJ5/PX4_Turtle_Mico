/****************************************************************************
 *
 *   Copyright (c) 2019-2022 PX4 Development Team. All rights reserved.
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

#include "DShot.h"

#include <px4_arch/io_timer.h>

#include <px4_platform_common/sem.hpp>
#include <matrix/matrix/math.hpp>

char DShot::_telemetry_device[] {};
bool DShot::_telemetry_swap_rxtx{false};
px4::atomic_bool DShot::_request_telemetry_init{false};

DShot::DShot() :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)
{
	_mixing_output.setAllDisarmedValues(DSHOT_DISARM_VALUE);
	_mixing_output.setAllMinValues(DSHOT_MIN_THROTTLE);
	_mixing_output.setAllMaxValues(DSHOT_MAX_THROTTLE);

	// Avoid using the PWM failsafe params
	_mixing_output.setAllFailsafeValues(UINT16_MAX);
}

DShot::~DShot()
{
	// make sure outputs are off
	up_dshot_arm(false);

	perf_free(_cycle_perf);
	perf_free(_bdshot_rpm_perf);
	perf_free(_dshot_telem_perf);

	delete _telemetry;
}

int DShot::init()
{
	_output_mask = (1u << _num_outputs) - 1;

	// Getting initial parameter values
	update_params();

	ScheduleNow();

	return OK;
}

int DShot::task_spawn(int argc, char *argv[])
{
	DShot *instance = new DShot();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

void DShot::enable_dshot_outputs(const bool enabled)
{
	if (enabled && !_outputs_initialized) {
		unsigned int dshot_frequency = 0;
		uint32_t dshot_frequency_param = 0;

		for (int timer = 0; timer < MAX_IO_TIMERS; ++timer) {
			uint32_t channels = io_timer_get_group(timer);

			if (channels == 0) {
				continue;
			}

			char param_name[17];
			snprintf(param_name, sizeof(param_name), "%s_TIM%u", _mixing_output.paramPrefix(), timer);

			int32_t tim_config = 0;
			param_t handle = param_find(param_name);
			param_get(handle, &tim_config);
			unsigned int dshot_frequency_request = 0;

			if (tim_config == -5) {
				dshot_frequency_request = DSHOT150;

			} else if (tim_config == -4) {
				dshot_frequency_request = DSHOT300;

			} else if (tim_config == -3) {
				dshot_frequency_request = DSHOT600;

			} else {
				_output_mask &= ~channels; // don't use for dshot
			}

			if (dshot_frequency_request != 0) {
				if (dshot_frequency != 0 && dshot_frequency != dshot_frequency_request) {
					PX4_WARN("Only supporting a single frequency, adjusting param %s", param_name);
					param_set_no_notification(handle, &dshot_frequency_param);

				} else {
					dshot_frequency = dshot_frequency_request;
					dshot_frequency_param = tim_config;
				}
			}
		}

		_bidirectional_dshot_enabled = _param_bidirectional_enable.get();

		int ret = up_dshot_init(_output_mask, dshot_frequency, _bidirectional_dshot_enabled);

		if (ret < 0) {
			PX4_ERR("up_dshot_init failed (%i)", ret);
			return;
		}

		_output_mask = ret;

		// disable unused functions
		for (unsigned i = 0; i < _num_outputs; ++i) {
			if (((1 << i) & _output_mask) == 0) {
				_mixing_output.disableFunction(i);

			}
		}

		if (_output_mask == 0) {
			// exit the module if no outputs used
			request_stop();
			return;
		}

		_outputs_initialized = true;
	}

	if (_outputs_initialized) {
		up_dshot_arm(enabled);
		_outputs_on = enabled;
	}
}

void DShot::update_num_motors()
{
	int motor_count = 0;

	for (unsigned i = 0; i < _num_outputs; ++i) {
		if (_mixing_output.isFunctionSet(i)) {
			_actuator_functions[motor_count] = (uint8_t)_mixing_output.outputFunction(i);
			++motor_count;
		}
	}

	_num_motors = motor_count;
}

void DShot::init_telemetry(const char *device, bool swap_rxtx)
{
	if (!_telemetry) {
		_telemetry = new DShotTelemetry{};

		if (!_telemetry) {
			PX4_ERR("alloc failed");
			return;
		}
	}

	if (device != NULL) {
		int ret = _telemetry->init(device, swap_rxtx);

		if (ret != 0) {
			PX4_ERR("telemetry init failed (%i)", ret);
		}
	}

	update_num_motors();
}

int DShot::handle_new_telemetry_data(const int telemetry_index, const DShotTelemetry::EscData &data, bool ignore_rpm)
{
	int ret = 0;
	// fill in new motor data
	esc_status_s &esc_status = esc_status_pub.get();

	if (telemetry_index < esc_status_s::CONNECTED_ESC_MAX) {
		esc_status.esc_online_flags |= 1 << telemetry_index;

		esc_status.esc[telemetry_index].actuator_function = _actuator_functions[telemetry_index];

		if (!ignore_rpm) {
			// If we also have bidirectional dshot, we use rpm and timestamps from there.
			esc_status.esc[telemetry_index].timestamp       = data.time;
			esc_status.esc[telemetry_index].esc_rpm         = (static_cast<int>(data.erpm) * 100) /
					(_param_mot_pole_count.get() / 2);
		}

		esc_status.esc[telemetry_index].esc_voltage     = static_cast<float>(data.voltage) * 0.01f;
		esc_status.esc[telemetry_index].esc_current     = static_cast<float>(data.current) * 0.01f;
		esc_status.esc[telemetry_index].esc_temperature = static_cast<float>(data.temperature);
		// TODO: accumulate consumption and use for battery estimation
	}

	// publish when motor index wraps (which is robust against motor timeouts)
	if (telemetry_index <= _last_telemetry_index) {
		esc_status.timestamp = hrt_absolute_time();
		esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_DSHOT;
		esc_status.esc_count = _num_motors;
		++esc_status.counter;

		ret = 1; // Indicate we wrapped, so we publish data
	}

	_last_telemetry_index = telemetry_index;

	perf_count(_dshot_telem_perf);

	return ret;
}

void DShot::publish_esc_status(void)
{
	esc_status_s &esc_status = esc_status_pub.get();
	int telemetry_index = 0;

	// clear data of the esc that are offline
	for (int index = 0; (index < _last_telemetry_index); index++) {
		if ((esc_status.esc_online_flags & (1 << index)) == 0) {
			memset(&esc_status.esc[index], 0, sizeof(struct esc_report_s));
		}
	}

	// FIXME: mark all UART Telemetry ESC's as online, otherwise commander complains even for a single dropout
	esc_status.esc_count = _num_motors;
	esc_status.esc_online_flags = (1 << esc_status.esc_count) - 1;
	esc_status.esc_armed_flags = (1 << esc_status.esc_count) - 1;

	if (_bidirectional_dshot_enabled) {
		for (unsigned i = 0; i < _num_outputs; i++) {
			if (_mixing_output.isFunctionSet(i)) {
				if (up_bdshot_channel_status(i)) {
					esc_status.esc_online_flags |= 1 << i;

				} else {
					esc_status.esc_online_flags &= ~(1 << i);
				}

				++telemetry_index;
			}
		}
	}

	if (!esc_status_pub.advertised()) {
		esc_status_pub.advertise();

	} else {
		esc_status_pub.update();
	}

	// reset esc online flags
	esc_status.esc_online_flags = 0;
}

int DShot::handle_new_bdshot_erpm(void)
{
	int num_erpms = 0;
	int telemetry_index = 0;
	int erpm;
	esc_status_s &esc_status = esc_status_pub.get();

	esc_status.timestamp = hrt_absolute_time();
	esc_status.counter = _esc_status_counter++;
	esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_DSHOT;
	esc_status.esc_armed_flags = _outputs_on;

	// We wait until all are ready.
	if (up_bdshot_num_erpm_ready() < _num_motors) {
		return 0;
	}

	for (unsigned i = 0; i < _num_outputs; i++) {
		if (_mixing_output.isFunctionSet(i)) {
			if (up_bdshot_get_erpm(i, &erpm) == 0) {
				num_erpms++;
				esc_status.esc_online_flags |= 1 << telemetry_index;
				esc_status.esc[telemetry_index].timestamp = hrt_absolute_time();
				esc_status.esc[telemetry_index].esc_rpm = (erpm * 100) / (_param_mot_pole_count.get() / 2);
				esc_status.esc[telemetry_index].actuator_function = _actuator_functions[telemetry_index];
			}

			++telemetry_index;
		}
	}

	perf_count(_bdshot_rpm_perf);

	return num_erpms;
}

int DShot::send_command_thread_safe(const dshot_command_t command, const int num_repetitions, const int motor_index)
{
	if (_recovery_busy.load()) {
		PX4_ERR("automatic turtle recovery is active");
		return -EBUSY;
	}

	Command cmd{};
	cmd.command = command;
	cmd.turtle_direction = command == DShot_cmd_spin_direction_normal || command == DShot_cmd_spin_direction_reversed;
	cmd.logical_motor = motor_index;
	cmd.requested_at = hrt_absolute_time();

	if (cmd.turtle_direction && (motor_index < 0 || motor_index >= actuator_motors_s::NUM_CONTROLS)) {
		return -EINVAL;
	}

	if (cmd.turtle_direction) {
		bool expected = false;

		if (!_turtle_busy.compare_exchange(&expected, true)) {
			PX4_ERR("temporary direction command busy");
			return -EBUSY;
		}

		_turtle_state.store(static_cast<int>(DShotTurtleCommand::State::Queued));

	} else if (_turtle_busy.load()) {
		PX4_ERR("wait for temporary direction command to finish");
		return -EBUSY;
	}

	if (motor_index == -1) {
		cmd.motor_mask = 0xff;

	} else {
		cmd.motor_mask = 1 << motor_index;
	}

	if (cmd.turtle_direction) {
		cmd.motor_mask = 0; // resolved and latched in the output worker
	}

	cmd.num_repetitions = num_repetitions;
	Command *expected_command = nullptr;

	if (!_new_command.compare_exchange(&expected_command, &cmd)) {
		if (cmd.turtle_direction) {
			_turtle_state.store(static_cast<int>(DShotTurtleCommand::State::Aborted));
			_turtle_busy.store(false);
		}

		return -EBUSY;
	}

	hrt_abstime timestamp_for_timeout = hrt_absolute_time();

	// wait until main thread processed it
	while (_new_command.load()) {

		if (hrt_elapsed_time(&timestamp_for_timeout) < 2_s) {
			px4_usleep(1000);

		} else {
			Command *pending = &cmd;

			if (_new_command.compare_exchange(&pending, nullptr) && cmd.turtle_direction) {
				_turtle_state.store(static_cast<int>(DShotTurtleCommand::State::TimedOut));
				_turtle_busy.store(false);
				PX4_ERR("temporary direction queue timeout");
				return -ETIMEDOUT;
			}

			PX4_WARN("DShot command timeout!");
		}
	}

	return 0;
}

void DShot::mixerChanged()
{
	update_num_motors();
}

turtle_recovery::OutputFrame DShot::recoveryFrame(uint16_t *outputs, unsigned count)
{
	using namespace turtle_recovery;
	static_assert(static_cast<uint8_t>(Action::Stop) == turtle_recovery_control_s::ACTION_STOP
		      && static_cast<uint8_t>(Action::Prepare) == turtle_recovery_control_s::ACTION_PREPARE
		      && static_cast<uint8_t>(Action::Drive) == turtle_recovery_control_s::ACTION_DRIVE, "Recovery action mismatch");
	static_assert(static_cast<uint8_t>(OutputState::Idle) == turtle_recovery_feedback_s::STATE_IDLE
		      && static_cast<uint8_t>(OutputState::Ready) == turtle_recovery_feedback_s::STATE_READY
		      && static_cast<uint8_t>(OutputState::Driving) == turtle_recovery_feedback_s::STATE_DRIVING
		      && static_cast<uint8_t>(OutputState::Restoring) == turtle_recovery_feedback_s::STATE_RESTORING,
		      "Recovery feedback mismatch");
	_recovery_control_sub.update(&_recovery_control);
	_recovery_manual_sub.update(&_recovery_manual);
	_recovery_attitude_sub.update(&_recovery_attitude);
	_recovery_gyro_sub.update(&_recovery_gyro);
	_recovery_land_sub.update(&_recovery_land);
	_recovery_status_sub.update(&_recovery_status);
	const uint64_t now = hrt_absolute_time();
	const auto fresh = [now](uint64_t t, uint64_t limit) { return t != 0 && now >= t && now - t < limit; };
	const auto &armed = _mixing_output.armed();
	OutputPermission p{};
	p.enabled = _turtle_enabled;
	p.stop_safe = fresh(armed.timestamp, 2000000) && !armed.armed && !armed.kill && !armed.lockdown
		      && !armed.termination && !armed.in_esc_calibration_mode;
	p.normal_outputs_stopped = true;
	p.available_mask = _output_mask;
	p.throttle_limit = _recovery_limit;
	unsigned seen[4] {};
	uint16_t all_mask = 0;
	bool extra_motor = false;

	for (unsigned i = 0; i < count; ++i) {
		const int motor = static_cast<int>(_mixing_output.outputFunction(i)) - static_cast<int>(OutputFunction::Motor1);

		if (motor >= 0 && motor < actuator_motors_s::NUM_CONTROLS) {
			p.normal_outputs_stopped &= outputs[i] == DSHOT_DISARM_VALUE;

			if (motor < 4 && i < 8 && (_output_mask & (1u << i))) {
				++seen[motor];
				all_mask |= 1u << i;

				if (motor == 0 || motor == 3) { p.right_mask |= 1u << i; }

			} else { extra_motor = true; }
		}
	}

	const bool mapping_valid = !extra_motor && seen[0] == 1 && seen[1] == 1 && seen[2] == 1 && seen[3] == 1;

	if (!mapping_valid || !_recovery_geometry) { p.right_mask = 0; }

	const float aux[] {_recovery_manual.aux1, _recovery_manual.aux2, _recovery_manual.aux3,
			   _recovery_manual.aux4, _recovery_manual.aux5, _recovery_manual.aux6
			  };
	const float selected_aux = _recovery_aux >= 1 && _recovery_aux <= 6 ? aux[_recovery_aux - 1] : NAN;
	matrix::Quatf q{_recovery_attitude.q};
	const bool attitude_valid = fresh(_recovery_attitude.timestamp_sample, 100000)
				    && q.isAllFinite() && fabsf(q.norm() - 1.f) < 0.02f;

	if (attitude_valid) { q.normalize(); }

	const matrix::Dcmf rotation{q};
	const matrix::Vector3f gyro{_recovery_gyro.xyz};
	const bool rc_valid = _recovery_manual.valid && _recovery_manual.data_source == manual_control_setpoint_s::SOURCE_RC
			      && std::isfinite(_rc_loss_timeout) && _rc_loss_timeout > 0.f
			      && fresh(_recovery_manual.timestamp_sample, static_cast<uint64_t>(_rc_loss_timeout * 1e6f))
			      && PX4_ISFINITE(selected_aux) && selected_aux > 0.5f && selected_aux <= 1.f
			      && PX4_ISFINITE(_recovery_manual.throttle) && _recovery_manual.throttle < -0.8f;
	const bool gyro_valid = fresh(_recovery_gyro.timestamp_sample, 100000) && gyro.isAllFinite() && gyro.norm() < 7.f;
	const bool vehicle_valid = fresh(_recovery_status.timestamp, 2000000)
				   && _recovery_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
				   && (!_recovery_status.safety_button_available || _recovery_status.safety_off)
				   && _recovery_status.power_input_valid && !_recovery_status.calibration_enabled
				   && !_recovery_status.rc_calibration_in_progress && !_recovery_status.failsafe
				   && (_recovery_status.failure_detector_status & ~3u) == 0;
	p.live = rc_valid && attitude_valid && gyro_valid && vehicle_valid
		 && !_param_dshot_3d_enable.get() && _reversible_outputs == 0;
	p.entry = p.live && rotation(2, 2) < -0.8660254f && fabsf(rotation(2, 0)) < 0.35f
		  && gyro.norm() < 0.174533f && fresh(_recovery_land.timestamp, 2000000) && _recovery_land.landed
		  && !_current_command.valid() && _new_command.load() == nullptr && _manual_dirty_mask == 0;

	if (_turtle_enabled && !_recovery_initialised && !_recovery.busy() && mapping_valid
	    && !_current_command.valid() && _new_command.load() == nullptr && p.stop_safe && p.normal_outputs_stopped) {
		_recovery.restore(now, all_mask);
		_recovery_initialised = true;
	}

	Request request{};
	request.timestamp = _recovery_control.timestamp;
	request.session = _recovery_control.session_id;
	request.action = static_cast<Action>(_recovery_control.action);
	request.throttle = _recovery_control.throttle;

	if (attitude_valid && rotation(2, 2) > 0.9063f && _recovery.state() == OutputState::Driving) {
		request.action = Action::Stop; // independent upright stop guard
	}

	const auto frame = _recovery.update(now, request, p);
	_recovery_busy.store(_recovery.busy() || (_turtle_enabled && !_recovery_initialised));
	_recovery_throttle = frame.throttle;
	return frame;
}

void DShot::publishRecoveryFeedback()
{
	turtle_recovery_feedback_s feedback{};
	feedback.timestamp = hrt_absolute_time();
	feedback.session_id = _recovery.session();
	feedback.state = static_cast<uint8_t>(_recovery.state());
	feedback.result = _recovery.result();
	feedback.busy = _recovery.busy() || _turtle_busy.load() || (_turtle_enabled && !_recovery_initialised);
	feedback.direction_dirty = _recovery.dirty() || _manual_dirty_mask != 0 || (_turtle_enabled && !_recovery_initialised);
	feedback.throttle = _recovery_throttle;
	_recovery_feedback_pub.publish(feedback);
}

bool DShot::updateOutputs(uint16_t outputs[MAX_ACTUATORS],
			  unsigned num_outputs, unsigned num_control_groups_updated)
{
	if (!_outputs_on) {
		return false;
	}

	int requested_telemetry_index = -1;

	if (_telemetry) {
		requested_telemetry_index = _telemetry->getRequestMotorIndex();
	}

	int telemetry_index = 0;
	const auto recovery_frame = recoveryFrame(outputs, num_outputs);
	const bool turtle_direction = _current_command.valid() && _current_command.turtle_direction;
	bool send_turtle_frame = false;

	if (turtle_direction) {
		// Resolve the logical Motor N function to an enabled physical DShot pin.
		unsigned matches = 0;
		uint8_t target_mask = 0;
		bool all_motors_stopped = true;

		for (unsigned i = 0; i < num_outputs; ++i) {
			const int function = static_cast<int>(_mixing_output.outputFunction(i));

			if (function >= static_cast<int>(OutputFunction::Motor1) && function <= static_cast<int>(OutputFunction::MotorMax)) {
				all_motors_stopped &= outputs[i] == DSHOT_DISARM_VALUE;
			}

			if (function == static_cast<int>(OutputFunction::Motor1) + _current_command.logical_motor) {
				++matches;

				if (i < 8 && (_output_mask & (1u << i))) {
					target_mask = 1u << i;
				}
			}
		}

		const auto &armed = _mixing_output.armed();
		const bool permitted = !armed.armed && !armed.kill && !armed.lockdown && !armed.termination
				       && !armed.in_esc_calibration_mode && !_param_dshot_3d_enable.get() && _reversible_outputs == 0;

		if (_current_command.motor_mask == 0) {
			_current_command.motor_mask = target_mask;
		}

		send_turtle_frame = _turtle_command.update(hrt_absolute_time(), permitted,
				    matches == 1 && target_mask != 0 && target_mask == _current_command.motor_mask, all_motors_stopped);
	}

	for (int i = 0; i < (int)num_outputs; i++) {
		if (recovery_frame.kind != turtle_recovery::FrameKind::None) {
			outputs[i] = DSHOT_DISARM_VALUE;

			if ((recovery_frame.mask & (1u << i)) && recovery_frame.kind == turtle_recovery::FrameKind::Throttle
			    && recovery_frame.throttle > 0.f) {
				outputs[i] = 1 + static_cast<uint16_t>(recovery_frame.throttle * (DSHOT_MAX_THROTTLE - 1));
			}

			if ((recovery_frame.mask & (1u << i)) && (recovery_frame.kind == turtle_recovery::FrameKind::Reverse
					|| recovery_frame.kind == turtle_recovery::FrameKind::Restore)) {
				up_dshot_motor_command(i, recovery_frame.kind == turtle_recovery::FrameKind::Reverse
						       ? DShot_cmd_spin_direction_reversed : DShot_cmd_spin_direction_normal, true);

			} else if (outputs[i] == DSHOT_DISARM_VALUE) {
				up_dshot_motor_command(i, DShot_cmd_motor_stop, false);

			} else {
				up_dshot_motor_data_set(i, outputs[i], false);
			}

			continue;
		}

		uint16_t output = outputs[i];

		if (output == DSHOT_DISARM_VALUE) {

			if (_current_command.valid() && (!turtle_direction || send_turtle_frame)
			    && (_current_command.motor_mask & (1 << i))) {
				up_dshot_motor_command(i, _current_command.command, true);

			} else {
				up_dshot_motor_command(i, DShot_cmd_motor_stop, telemetry_index == requested_telemetry_index);
			}

		} else {

			if (_param_dshot_3d_enable.get() || (_reversible_outputs & (1u << i))) {
				output = convert_output_to_3d_scaling(output);
			}

			up_dshot_motor_data_set(i, math::min(output, static_cast<uint16_t>(DSHOT_MAX_THROTTLE)),
						telemetry_index == requested_telemetry_index);
		}

		telemetry_index += _mixing_output.isFunctionSet(i);
	}

	// Decrement the command counter
	const auto previous_recovery_state = _recovery.state();
	_recovery.frameSent(recovery_frame.kind);

	if (previous_recovery_state == turtle_recovery::OutputState::Restoring
	    && _recovery.state() == turtle_recovery::OutputState::Idle) {
		// Automatic restore affects the two recovery motors (all four on startup).
		_manual_dirty_mask &= _recovery.session() == 0 ? ~turtle_recovery::ALL_MOTORS : ~turtle_recovery::RIGHT_MOTORS;
	}

	if (turtle_direction) {
		if (send_turtle_frame) {
			if (_current_command.command == DShot_cmd_spin_direction_reversed) {
				_manual_dirty_mask |= 1u << _current_command.logical_motor;
			}

			_turtle_command.frameSent();
		}

		_turtle_state.store(static_cast<int>(_turtle_command.state()));

		if (_turtle_command.finished()) {
			if (_turtle_command.state() == DShotTurtleCommand::State::Sent
			    && _current_command.command == DShot_cmd_spin_direction_normal) {
				_manual_dirty_mask &= ~(1u << _current_command.logical_motor);
			}

			PX4_INFO("Temporary direction Motor %d: %s", _current_command.logical_motor + 1,
				 DShotTurtleCommand::name(_turtle_command.state()));
			_current_command.clear();
			_turtle_busy.store(false);
		}

	} else if (_current_command.valid()) {
		--_current_command.num_repetitions;

		// Queue a save command after the burst if save has been requested
		if (_current_command.num_repetitions == 0 && _current_command.save) {
			_current_command.save = false;
			_current_command.num_repetitions = 10;
			_current_command.command = dshot_command_t::DShot_cmd_save_settings;
		}
	}

	up_dshot_trigger();

	return true;
}

uint16_t DShot::convert_output_to_3d_scaling(uint16_t output)
{
	// DShot 3D splits the throttle ranges in two.
	// This is in terms of DShot values, code below is in terms of actuator_output
	// Direction 1) 48 is the slowest, 1047 is the fastest.
	// Direction 2) 1049 is the slowest, 2047 is the fastest.
	if (output >= _param_dshot_3d_dead_l.get() && output < _param_dshot_3d_dead_h.get()) {
		return DSHOT_DISARM_VALUE;
	}

	bool upper_range = output >= 1000;

	if (upper_range) {
		output -= 1000;

	} else {
		output = 999 - output; // lower range is inverted
	}

	float max_output = 999.f;
	float min_output = max_output * _param_dshot_min.get();
	output = math::min(max_output, (min_output + output * (max_output - min_output) / max_output));

	if (upper_range) {
		output += 1000;
	}

	return output;
}

void DShot::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	_mixing_output.update();
	publishRecoveryFeedback();

	// update output status if armed or if mixer is loaded
	bool outputs_on = true;

	if (_outputs_on != outputs_on) {
		enable_dshot_outputs(outputs_on);
	}

	if (_telemetry) {
		const int telem_update = _telemetry->update(_num_motors);

		if (telem_update >= 0) {
			const int need_to_publish = handle_new_telemetry_data(telem_update, _telemetry->latestESCData(),
						    _bidirectional_dshot_enabled);

			// We don't want to publish twice, once by telemetry and once by bidirectional dishot.
			if (!_bidirectional_dshot_enabled && need_to_publish) {
				publish_esc_status();
			}
		}
	}

	if (_bidirectional_dshot_enabled) {
		// Add bdshot data to esc status
		const int need_to_publish = handle_new_bdshot_erpm();

		if (need_to_publish) {
			publish_esc_status();
		}
	}

	if (_parameter_update_sub.updated()) {
		update_params();
	}

	// telemetry device update request?
	if (_request_telemetry_init.load()) {
		init_telemetry(_telemetry_device, _telemetry_swap_rxtx);
		_request_telemetry_init.store(false);
	}

	// new command?
	if (!_current_command.valid() && !_recovery.busy()) {
		Command *new_command = _new_command.load();

		if (new_command) {
			_current_command = *new_command;

			if (_current_command.turtle_direction) {
				_turtle_command.start(_current_command.requested_at);
			}

			_new_command.store(nullptr);
		}
	}

	handle_vehicle_commands();

	if (!_mixing_output.armed().armed) {
		if (_reversible_outputs != _mixing_output.reversibleOutputs()) {
			_reversible_outputs = _mixing_output.reversibleOutputs();
			update_params();
		}
	}

	// check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
	_mixing_output.updateSubscriptions(true);

	perf_end(_cycle_perf);
}

void DShot::handle_vehicle_commands()
{
	vehicle_command_s vehicle_command;

	while (!_current_command.valid() && _vehicle_command_sub.update(&vehicle_command)) {

		if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_CONFIGURE_ACTUATOR) {
			int function = (int)(vehicle_command.param5 + 0.5);

			if (function < 1000) {
				const int first_motor_function = 1; // from MAVLink ACTUATOR_OUTPUT_FUNCTION
				const int first_servo_function = 33;

				if (function >= first_motor_function && function < first_motor_function + actuator_test_s::MAX_NUM_MOTORS) {
					function = function - first_motor_function + actuator_test_s::FUNCTION_MOTOR1;

				} else if (function >= first_servo_function && function < first_servo_function + actuator_test_s::MAX_NUM_SERVOS) {
					function = function - first_servo_function + actuator_test_s::FUNCTION_SERVO1;

				} else {
					function = INT32_MAX;
				}

			} else {
				function -= 1000;
			}

			int type = (int)(vehicle_command.param1 + 0.5f);
			int index = -1;

			for (int i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; ++i) {
				if ((int)_mixing_output.outputFunction(i) == function) {
					index = i;
				}
			}

			vehicle_command_ack_s command_ack{};
			command_ack.command = vehicle_command.command;
			command_ack.target_system = vehicle_command.source_system;
			command_ack.target_component = vehicle_command.source_component;
			command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;

			if (_recovery_busy.load() || _manual_dirty_mask != 0) {
				command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;

			} else if (index != -1) {
				PX4_DEBUG("setting command: index: %i type: %i", index, type);
				_current_command.command = dshot_command_t::DShot_cmd_motor_stop;

				switch (type) {
				case 1: _current_command.command = dshot_command_t::DShot_cmd_beacon1; break;

				case 2: _current_command.command = dshot_command_t::DShot_cmd_3d_mode_on; break;

				case 3: _current_command.command = dshot_command_t::DShot_cmd_3d_mode_off; break;

				case 4: _current_command.command = dshot_command_t::DShot_cmd_spin_direction_1; break;

				case 5: _current_command.command = dshot_command_t::DShot_cmd_spin_direction_2; break;
				}

				if (_current_command.command == dshot_command_t::DShot_cmd_motor_stop) {
					PX4_WARN("unknown command: %i", type);

				} else {
					command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
					_current_command.motor_mask = 1 << index;
					_current_command.num_repetitions = 10;
					_current_command.save = true;
				}

			}

			command_ack.timestamp = hrt_absolute_time();
			_command_ack_pub.publish(command_ack);
		}
	}
}

void DShot::update_params()
{
	parameter_update_s pupdate;
	_parameter_update_sub.copy(&pupdate);

	updateParams();
	int32_t enabled = 0;
	param_get(param_find("TRTL_ENABLE"), &enabled);

	if (enabled != 0 && !_turtle_enabled) { _recovery_initialised = false; }

	_turtle_enabled = enabled != 0;
	param_get(param_find("TRTL_AUX"), &_recovery_aux);
	param_get(param_find("TRTL_MAX_THR"), &_recovery_limit);
	param_get(param_find("COM_RC_LOSS_T"), &_rc_loss_timeout);
	int32_t rotor_count = 0, airframe = -1;
	param_get(param_find("CA_ROTOR_COUNT"), &rotor_count);
	param_get(param_find("CA_AIRFRAME"), &airframe);
	_recovery_geometry = rotor_count == 4 && airframe == 0;

	for (unsigned i = 0; i < 4; ++i) {
		char name[17];
		float y = NAN;
		snprintf(name, sizeof(name), "CA_ROTOR%u_PY", i);
		param_get(param_find(name), &y);
		_recovery_geometry &= PX4_ISFINITE(y) && ((i == 0 || i == 3) ? y > 0.f : y < 0.f);
	}

	// we use a minimum value of 1, since 0 is for disarmed
	_mixing_output.setAllMinValues(math::constrain(static_cast<int>((_param_dshot_min.get() *
				       static_cast<float>(DSHOT_MAX_THROTTLE))),
				       DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE));

	// Do not use the minimum parameter for reversible outputs
	for (unsigned i = 0; i < _num_outputs; ++i) {
		if ((1 << i) & _reversible_outputs) {
			_mixing_output.minValue(i) = DSHOT_MIN_THROTTLE;
		}
	}
}

int DShot::custom_command(int argc, char *argv[])
{
	const char *verb = argv[0];

	int motor_index = -1; // select motor index, default: -1=all
	int myoptind = 1;
	bool swap_rxtx = false;
	const char *device_name = nullptr;
	const char *motor_argument = nullptr;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "m:xd:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'm': {
				motor_argument = myoptarg;
				char *end = nullptr;
				const long motor = strtol(myoptarg, &end, 10);

				if (end == myoptarg || *end != '\0' || motor < 0 || motor > 16) {
					return print_usage("invalid motor index");
				}

				motor_index = static_cast<int>(motor) - 1;
				break;
			}

		case 'x':
			swap_rxtx = true;
			break;

		case 'd':
			device_name = myoptarg;
			break;

		default:
			return print_usage("unrecognized flag");
		}
	}

	if (!strcmp(verb, "telemetry")) {
		if (device_name) {
			// telemetry can be requested before the module is started
			strncpy(_telemetry_device, device_name, sizeof(_telemetry_device) - 1);
			_telemetry_device[sizeof(_telemetry_device) - 1] = '\0';
			_telemetry_swap_rxtx = swap_rxtx;
			_request_telemetry_init.store(true);
		}

		return 0;
	}

	struct VerbCommand {
		const char *name;
		dshot_command_t command;
		int num_repetitions;
	};

	constexpr VerbCommand commands[] = {
		{"turtle_reverse", DShot_cmd_spin_direction_reversed, 10},
		{"turtle_normal", DShot_cmd_spin_direction_normal, 10},
		{"reverse", DShot_cmd_spin_direction_2, 10},
		{"normal", DShot_cmd_spin_direction_1, 10},
		{"save", DShot_cmd_save_settings, 10},
		{"3d_on", DShot_cmd_3d_mode_on, 10},
		{"3d_off", DShot_cmd_3d_mode_off, 10},
		{"beep1", DShot_cmd_beacon1, 1},
		{"beep2", DShot_cmd_beacon2, 1},
		{"beep3", DShot_cmd_beacon3, 1},
		{"beep4", DShot_cmd_beacon4, 1},
		{"beep5", DShot_cmd_beacon5, 1},
	};

	for (unsigned i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i) {
		if (!strcmp(verb, commands[i].name)) {
			const bool turtle_direction = commands[i].command == DShot_cmd_spin_direction_normal
						      || commands[i].command == DShot_cmd_spin_direction_reversed;

			if (turtle_direction) {
				char *end = nullptr;
				const long motor = motor_argument ? strtol(motor_argument, &end, 10) : 0;

				if (!motor_argument || end == motor_argument || *end != '\0' || motor < 1 || motor > actuator_motors_s::NUM_CONTROLS) {
					return print_usage("specify one logical Motor with -m 1..12; remove all propellers");
				}
			}

			if (!is_running()) {
				PX4_ERR("module not running");
				return -1;
			}

			const int result = get_instance()->send_command_thread_safe(commands[i].command, commands[i].num_repetitions, motor_index);

			if (result == PX4_OK && turtle_direction) {
				PX4_INFO("Queued; check dshot status for TX completion, then verify direction with props removed");
			}

			return result;
		}
	}

	if (!is_running()) {
		int ret = DShot::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int DShot::print_status()
{
	PX4_INFO("Outputs initialized: %s", _outputs_initialized ? "yes" : "no");
	PX4_INFO("Outputs used: 0x%" PRIx32, _output_mask);
	PX4_INFO("Outputs on: %s", _outputs_on ? "yes" : "no");
	PX4_INFO("Temporary direction: %s", DShotTurtleCommand::name(static_cast<DShotTurtleCommand::State>(_turtle_state.load())));
	PX4_INFO("Automatic recovery: state %u, busy %s, direction dirty %s, throttle %.3f",
		 static_cast<unsigned>(_recovery.state()), _recovery_busy.load() ? "yes" : "no",
		 (_recovery.dirty() || _manual_dirty_mask != 0) ? "yes" : "no", (double)_recovery_throttle);
	perf_print_counter(_cycle_perf);
	perf_print_counter(_bdshot_rpm_perf);
	perf_print_counter(_dshot_telem_perf);

	_mixing_output.printStatus();

	if (_telemetry) {
		PX4_INFO("telemetry on: %s", _telemetry_device);
		_telemetry->printStatus();
	}

	/* Print dshot status */
	if (_bidirectional_dshot_enabled) {
		up_bdshot_status();
	}

	return 0;
}

int DShot::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This is the DShot output driver. It is similar to the fmu driver, and can be used as drop-in replacement
to use DShot as ESC communication protocol instead of PWM.

On startup, the module tries to occupy all available pins for DShot output.
It skips all pins already in use (e.g. by a camera trigger module).

It supports:
- DShot150, DShot300, DShot600
- telemetry via separate UART and publishing as esc_status message
- sending DShot commands via CLI

### Examples
Permanently reverse motor 1:
$ dshot reverse -m 1
$ dshot save -m 1
After saving, the reversed direction will be regarded as the normal one. So to reverse again repeat the same commands.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dshot", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");

	PRINT_MODULE_USAGE_COMMAND_DESCR("telemetry", "Enable Telemetry on a UART");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<device>", "UART device", false);
	PRINT_MODULE_USAGE_PARAM_FLAG('x', "Swap RX/TX pins", true);

	// DShot commands
	PRINT_MODULE_USAGE_COMMAND_DESCR("turtle_reverse", "Props off: temporary reverse (21), disarmed only, no save");
	PRINT_MODULE_USAGE_PARAM_INT('m', 1, 1, 12, "Logical Motor function (not output pin), required", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("turtle_normal", "Props off: restore saved direction (20), disarmed only");
	PRINT_MODULE_USAGE_PARAM_INT('m', 1, 1, 12, "Logical Motor function (not output pin), required", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("reverse", "Reverse motor direction");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("normal", "Normal motor direction");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("save", "Save current settings");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("3d_on", "Enable 3D mode");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("3d_off", "Disable 3D mode");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("beep1", "Send Beep pattern 1");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("beep2", "Send Beep pattern 2");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("beep3", "Send Beep pattern 3");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("beep4", "Send Beep pattern 4");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("beep5", "Send Beep pattern 5");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int dshot_main(int argc, char *argv[])
{
	return DShot::main(argc, argv);
}
