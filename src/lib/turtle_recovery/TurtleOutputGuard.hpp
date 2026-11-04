// SPDX-License-Identifier: BSD-3-Clause
//驱动状态机：停机等待、反转、限幅、超时、恢复保存转向
// Copyright (c) 2026 PX4 Development Team. All rights reserved.
#pragma once
#include "TurtleProtocol.hpp"
#include <cmath>

namespace turtle_recovery
{
// Runs in the DShot worker. Loss of the controller always stops thrust and
// restores direction locally; no controller heartbeat is needed for cleanup.
class TurtleOutputGuard
{
public:
	void restore(uint64_t now, uint16_t mask)
	{
		_session = 0; // startup/reenable restore covers all mapped motors
		_result = 0;
		_mask = mask;
		_dirty = true; // conservative: direction unknown after a controller reboot
		_state = OutputState::StopRestore;
		_stopped_since = 0;
		_last_now = now;
	}

	OutputFrame update(uint64_t now, const Request &request, const OutputPermission &p)
	{
		const bool fresh = request.timestamp != 0 && now >= request.timestamp && now - request.timestamp < LEASE_US;

		if (_state == OutputState::Idle) {
			if (!fresh || request.session == 0 || request.session == _session || request.action != Action::Prepare
			    || !p.enabled || !p.stop_safe || !p.live || !p.entry || !p.normal_outputs_stopped || p.right_mask == 0) {
				return {};
			}

			_session = request.session;
			_mask = p.right_mask;
			_state = OutputState::StopReverse;
			_stopped_since = 0;
			_result = 0;
		}

		const bool restoring = _state == OutputState::StopRestore || _state == OutputState::Restoring;

		if (!restoring) {
			const bool abort = !fresh || request.session != _session || request.action == Action::Stop
					   || !p.enabled || !p.stop_safe || !p.live || !p.normal_outputs_stopped
					   || p.right_mask != _mask || now < _last_now;

			if (abort || (_state == OutputState::Driving && now - _drive_start >= 3000000)) {
				_result = (fresh && request.session == _session && request.action == Action::Stop) ? 0 : 1;
				_state = OutputState::StopRestore;
				_stopped_since = 0;
			}
		}

		_last_now = now;
		OutputFrame frame{FrameKind::Stop, _mask, 0.f};

		if (_state == OutputState::StopReverse || _state == OutputState::StopRestore) {
			if (!p.stop_safe || (_mask & p.available_mask) != _mask) {
				_stopped_since = 0;
				return frame;
			}

			if (_stopped_since == 0) { _stopped_since = now; }

			if (now - _stopped_since < 500000) { return frame; }

			_state = _state == OutputState::StopReverse ? OutputState::Reversing : OutputState::Restoring;
			_remaining = 10;
		}

		if (_state == OutputState::Reversing || _state == OutputState::Restoring) {
			if (!p.stop_safe || (_mask & p.available_mask) != _mask) { return frame; }

			frame.kind = _state == OutputState::Reversing ? FrameKind::Reverse : FrameKind::Restore;
			return frame;
		}

		if (_state == OutputState::Ready && request.action == Action::Drive) {
			_state = OutputState::Driving;
			_drive_start = now;
		}

		if (_state == OutputState::Driving) {
			if (request.action != Action::Drive || !std::isfinite(request.throttle) || request.throttle < 0.f
			    || !std::isfinite(p.throttle_limit) || p.throttle_limit <= 0.f) {
				_state = OutputState::StopRestore;
				_stopped_since = 0;
				_result = 1;
				return frame;
			}

			frame.kind = FrameKind::Throttle;
			frame.throttle = fminf(request.throttle, fminf(p.throttle_limit, HARD_MAX_THROTTLE));
		}

		return frame;
	}

	void frameSent(FrameKind kind)
	{
		if (_state == OutputState::Reversing && kind == FrameKind::Reverse) {
			_dirty = true; // even an interrupted burst may have reached an ESC

			if (--_remaining == 0) { _state = OutputState::Ready; }

		} else if (_state == OutputState::Restoring && kind == FrameKind::Restore) {
			if (--_remaining == 0) {
				_dirty = false;
				_state = OutputState::Idle;
			}
		}
	}

	bool busy() const { return _state != OutputState::Idle; }
	bool dirty() const { return _dirty; }
	uint64_t session() const { return _session; }
	OutputState state() const { return _state; }
	uint8_t result() const { return _result; }
	uint16_t mask() const { return _mask; }
private:
	OutputState _state{OutputState::Idle};
	uint64_t _session{0}, _last_now{0}, _stopped_since{0}, _drive_start{0};
	uint16_t _mask{0};
	unsigned _remaining{0};
	bool _dirty{false};
	uint8_t _result{0};
};
} // namespace turtle_recovery
