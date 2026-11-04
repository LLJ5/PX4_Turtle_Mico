// SPDX-License-Identifier: BSD-3-Clause
//共享状态、请求结构、电机掩码、油门硬上限
// Copyright (c) 2026 PX4 Development Team. All rights reserved.
#pragma once
#include <cstdint>

namespace turtle_recovery
{
static constexpr uint16_t RIGHT_MOTORS = 0x9; // logical Motor 1 and Motor 4
static constexpr uint16_t ALL_MOTORS = 0xf;
static constexpr uint64_t LEASE_US = 200000;
static constexpr float HARD_MAX_THROTTLE = 0.8f;
enum class Action : uint8_t { Stop = 0, Prepare = 1, Drive = 2 };
enum class OutputState : uint8_t { Idle, StopReverse, Reversing, Ready, Driving, StopRestore, Restoring };
enum class FrameKind { None, Stop, Reverse, Throttle, Restore };
struct Request {
	uint64_t timestamp{0};
	uint64_t session{0};
	Action action{Action::Stop};
	float throttle{0.f};
};
struct OutputPermission {
	bool enabled{false};
	bool stop_safe{false}; // disarmed, no kill/lockdown/termination/calibration
	bool live{false};      // fresh RC HIGH, low throttle, valid attitude and gyro
	bool entry{false};     // also inverted, landed and stationary
	bool normal_outputs_stopped{false};
	uint16_t right_mask{0}; // physical DShot pins resolved from Motor functions
	uint16_t available_mask{0};
	float throttle_limit{0.f};
};
struct OutputFrame {
	FrameKind kind{FrameKind::None};
	uint16_t mask{0};
	float throttle{0.f};
};
} // namespace turtle_recovery
