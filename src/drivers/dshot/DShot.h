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
#pragma once

#include <drivers/drv_dshot.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

#include "DShotTelemetry.h"
#include "DShotTurtleCommand.hpp"
#include <turtle_recovery/TurtleOutputGuard.hpp>
#include <uORB/topics/turtle_recovery_control.h>
#include <uORB/topics/turtle_recovery_feedback.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

#if !defined(DIRECT_PWM_OUTPUT_CHANNELS)
#  error "board_config.h needs to define DIRECT_PWM_OUTPUT_CHANNELS"
#endif

/** Dshot PWM frequency, Hz */
static constexpr unsigned int DSHOT150  =  150000u;
static constexpr unsigned int DSHOT300  =  300000u;
static constexpr unsigned int DSHOT600  =  600000u;

static constexpr int DSHOT_DISARM_VALUE = 0;
static constexpr int DSHOT_MIN_THROTTLE = 1;
static constexpr int DSHOT_MAX_THROTTLE = 1999;

class DShot final : public ModuleBase<DShot>, public OutputModuleInterface
{
public:
	DShot();
	~DShot() override;

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	int init();

	void mixerChanged() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/**
	 * Send a dshot command to one or all motors
	 * This is expected to be called from another thread.
	 * @param num_repetitions number of times to repeat, set at least to 1
	 * @param motor_index index or -1 for all
	 * @return 0 on success, <0 error otherwise
	 */
	int send_command_thread_safe(const dshot_command_t command, const int num_repetitions, const int motor_index);

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	bool telemetry_enabled() const { return _telemetry != nullptr; }

	bool updateOutputs(uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

private:

	/** Disallow copy construction and move assignment. */
	DShot(const DShot &) = delete;
	DShot operator=(const DShot &) = delete;

	enum class DShotConfig {
		Disabled  = 0,
		DShot150  = 150,
		DShot300  = 300,
		DShot600  = 600,
	};

	struct Command {
		dshot_command_t command{};
		int num_repetitions{0};
		uint8_t motor_mask{0xff};
		bool save{false};
		bool turtle_direction{false};
		int logical_motor{-1};
		hrt_abstime requested_at{0};

		bool valid() const { return num_repetitions > 0; }
		void clear() { num_repetitions = 0; turtle_direction = false; save = false; }
	};

	int _last_telemetry_index{-1};
	uint8_t _actuator_functions[esc_status_s::CONNECTED_ESC_MAX] {};

	void enable_dshot_outputs(const bool enabled);

	void init_telemetry(const char *device, bool swap_rxtx);

	int handle_new_telemetry_data(const int telemetry_index, const DShotTelemetry::EscData &data, bool ignore_rpm);

	void publish_esc_status(void);

	int handle_new_bdshot_erpm(void);

	void Run() override;

	void update_params();

	void update_num_motors();

	void handle_vehicle_commands();

	uint16_t convert_output_to_3d_scaling(uint16_t output);
	turtle_recovery::OutputFrame recoveryFrame(uint16_t *outputs, unsigned count);
	void publishRecoveryFeedback();

	MixingOutput _mixing_output{PARAM_PREFIX, DIRECT_PWM_OUTPUT_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};
	uint32_t _reversible_outputs{};

	DShotTelemetry *_telemetry{nullptr};

	uORB::PublicationMultiData<esc_status_s> esc_status_pub{ORB_ID(esc_status)};

	static char _telemetry_device[20];
	static bool _telemetry_swap_rxtx;
	static px4::atomic_bool _request_telemetry_init;

	px4::atomic<Command *> _new_command{nullptr};


	bool _outputs_initialized{false};
	bool _outputs_on{false};
	bool _bidirectional_dshot_enabled{false};

	static constexpr unsigned _num_outputs{DIRECT_PWM_OUTPUT_CHANNELS};
	uint32_t _output_mask{0};

	int _num_motors{0};

	perf_counter_t	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_bdshot_rpm_perf{perf_alloc(PC_COUNT, MODULE_NAME": bdshot rpm")};
	perf_counter_t	_dshot_telem_perf{perf_alloc(PC_COUNT, MODULE_NAME": dshot telem")};

	Command _current_command{};
	DShotTurtleCommand _turtle_command;
	px4::atomic_bool _turtle_busy{false};
	px4::atomic<int> _turtle_state{static_cast<int>(DShotTurtleCommand::State::Idle)};
	turtle_recovery::TurtleOutputGuard _recovery;
	px4::atomic_bool _recovery_busy{false};
	uint16_t _manual_dirty_mask{0};
	bool _turtle_enabled{false}, _recovery_initialised{false}, _recovery_geometry{false};
	int32_t _recovery_aux{1};
	float _recovery_limit{0.12f}, _rc_loss_timeout{0.5f}, _recovery_throttle{0.f};
	uORB::Subscription _recovery_control_sub{ORB_ID(turtle_recovery_control)};
	uORB::Subscription _recovery_manual_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _recovery_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _recovery_gyro_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _recovery_land_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _recovery_status_sub{ORB_ID(vehicle_status)};
	turtle_recovery_control_s _recovery_control{};
	manual_control_setpoint_s _recovery_manual{};
	vehicle_attitude_s _recovery_attitude{};
	vehicle_angular_velocity_s _recovery_gyro{};
	vehicle_land_detected_s _recovery_land{};
	vehicle_status_s _recovery_status{};
	uORB::Publication<turtle_recovery_feedback_s> _recovery_feedback_pub{ORB_ID(turtle_recovery_feedback)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_ack_s> _command_ack_pub{ORB_ID(vehicle_command_ack)};
	uint16_t _esc_status_counter{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::DSHOT_MIN>)    _param_dshot_min,
		(ParamBool<px4::params::DSHOT_3D_ENABLE>) _param_dshot_3d_enable,
		(ParamInt<px4::params::DSHOT_3D_DEAD_H>) _param_dshot_3d_dead_h,
		(ParamInt<px4::params::DSHOT_3D_DEAD_L>) _param_dshot_3d_dead_l,
		(ParamInt<px4::params::MOT_POLE_COUNT>) _param_mot_pole_count,
		(ParamBool<px4::params::DSHOT_BIDIR_EN>) _param_bidirectional_enable
	)
};
