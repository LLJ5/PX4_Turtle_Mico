// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team. All rights reserved.

#pragma once

#include <cstdint>

// 仅管理停机等待和命令发送，不产生电机油门，也不代表 ESC 已确认转向。
class DShotTurtleCommand
{
public:
	enum class State { Idle, Queued, WaitStop, Sending, Sent, Aborted, TimedOut };
	static constexpr uint64_t STOP_WAIT_US = 500000;
	static constexpr uint64_t TIMEOUT_US = 3000000;
	static constexpr unsigned REPETITIONS = 10;

	void start(uint64_t requested_at)
	{
		_started = requested_at;
		_stopped_since = 0;
		_remaining = REPETITIONS;
		_state = State::WaitStop;
	}

	bool update(uint64_t now, bool permitted, bool target_valid, bool all_motors_stopped)
	{
		if (_state != State::WaitStop && _state != State::Sending) {
			return false;
		}

		if (!permitted || !target_valid || now < _started) {
			_state = State::Aborted;
			return false;
		}

		if (now - _started >= TIMEOUT_US) {
			_state = State::TimedOut;
			return false;
		}

		if (!all_motors_stopped) {
			if (_state == State::Sending) {
				_state = State::Aborted;
			}

			_stopped_since = 0;
			return false;
		}

		if (_state == State::WaitStop) {
			if (_stopped_since == 0) {
				_stopped_since = now;
			}

			if (now - _stopped_since < STOP_WAIT_US) {
				return false;
			}

			_state = State::Sending;
		}

		return true;
	}

	void frameSent()
	{
		if (_state == State::Sending && --_remaining == 0) {
			_state = State::Sent;
		}
	}

	State state() const { return _state; }
	bool finished() const { return _state == State::Sent || _state == State::Aborted || _state == State::TimedOut; }

	static const char *name(State state)
	{
		switch (state) {
		case State::Idle: return "idle";

		case State::Queued: return "queued";

		case State::WaitStop: return "waiting for stopped outputs";

		case State::Sending: return "sending";

		case State::Sent: return "TX complete (not ESC acknowledgement)";

		case State::Aborted: return "aborted (arming, output, mapping or mode changed)";

		case State::TimedOut: return "timed out";
		}

		return "unknown";
	}

private:
	uint64_t _started{0};
	uint64_t _stopped_since{0};
	unsigned _remaining{0};
	State _state{State::Idle};
};
