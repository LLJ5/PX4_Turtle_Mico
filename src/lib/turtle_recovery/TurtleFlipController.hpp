// SPDX-License-Identifier: BSD-3-Clause[TurtleFlipController.hpp](/home/luo/PX4-Autopilot/src/lib/turtle_recovery/TurtleFlipController.hpp)
//翻身状态机：确认、准备、出力、收油、成功与失败
// Copyright (c) 2026 PX4 Development Team. All rights reserved.
#pragma once
#include "TurtleProtocol.hpp"
#include <cmath>

namespace turtle_recovery
{
class TurtleFlipController
{
public:
	enum class State { Idle, Qualify, Prepare, Drive, Coast, Cleanup, Done, Failed };
	struct Input {
		uint64_t now{0};
		bool enabled{false}, safe{false}, low{false}, high{false}, entry{false};
		bool feedback_fresh{false}, backend_idle{false}, backend_ready{false}, backend_clean{false};
		uint64_t backend_session{0};
		float tilt{0.f}, phase{0.f}, roll_rate{0.f}, gyro_norm{0.f}; // degrees except phase (rad)
	};
	struct Settings {
		float max_throttle{0.12f}, target_rate{120.f}, release_tilt{75.f};
		float no_progress_timeout{1.2f}, drive_timeout{2.5f}; // seconds from Drive entry, including ramp
		float throttle_slew{0.25f}; // normalized throttle increase per second at zero roll rate
	};

	static bool validTiming(const Settings &settings)
	{
		return std::isfinite(settings.no_progress_timeout) && std::isfinite(settings.drive_timeout)
		       && settings.no_progress_timeout >= 0.5f && settings.no_progress_timeout <= 2.5f
		       && settings.drive_timeout >= 0.5f && settings.drive_timeout <= 2.8f
		       && settings.no_progress_timeout <= settings.drive_timeout;
	}

	static bool validSlew(const Settings &settings)
	{
		return std::isfinite(settings.throttle_slew) && settings.throttle_slew >= 0.1f && settings.throttle_slew <= 1.f;
	}

	Request update(const Input &in, const Settings &settings)
	{
		const float dt = _last_now != 0 && in.now >= _last_now ? fminf(0.05f, (in.now - _last_now) * 1e-6f) : 0.f;
		_last_now = in.now;
		const bool inactive = _state == State::Idle || _state == State::Done || _state == State::Failed;
		const bool timing_valid = validTiming(settings);
		const bool settings_valid = timing_valid && validSlew(settings);
		const char *invalid_settings_reason = !timing_valid ? "invalid recovery time settings" : "invalid recovery throttle slew";

		if (!in.safe || !in.enabled || !settings_valid) { _off_seen = false; }

		if (inactive) {
			if (!settings_valid) { _reason = invalid_settings_reason; }

			if (in.safe && settings_valid && in.low) {
				_off_seen = true;
				_state = State::Idle;
				_session = 0;
				_progress = 0.f;
				_reason = "waiting for RC HIGH";
			}

			if (in.enabled && in.safe && settings_valid && in.high && _off_seen) {
				_off_seen = false;

				if (in.entry && in.feedback_fresh && in.backend_idle && in.backend_clean) {
					_state = State::Qualify;
					_started = in.now;
					_reason = "qualifying inverted rest";

				} else {
					_state = State::Failed;
					_reason = "entry rejected: require inverted rest and clean backend";
				}
			}

			return request(in.now);
		}

		if (_state != State::Cleanup && (!settings_valid || !in.enabled || !in.safe || !in.high || !in.feedback_fresh)) {
			fail(!settings_valid ? invalid_settings_reason : "disabled, RC/attitude lost, switch off or arming");
		}

		if (_state == State::Qualify) {
			if (!in.entry || !in.backend_idle || !in.backend_clean) {
				_state = State::Failed;
				_reason = "inverted rest interrupted";

			} else if (in.now - _started >= 500000) {
				_session = in.now;
				_state = State::Prepare;
				_started = in.now;
				_reason = "reversing Motor 1 and Motor 4";
			}

		} else if (_state == State::Prepare) {
			if (!in.entry) {
				fail("inverted rest interrupted before drive");

			} else if (in.backend_session == _session && in.backend_ready) {
				_state = State::Drive;
				_started = in.now;
				_last_phase = in.phase;
				_progress = 0.f;
				_throttle = 0.f;
				_reason = "fixed positive roll";

			} else if (in.now - _started > 3000000) { fail("reverse preparation timeout"); }

		} else if (_state == State::Drive) {
			float delta = in.phase - _last_phase;

			while (delta > 3.14159265f) { delta -= 6.28318531f; }

			while (delta < -3.14159265f) { delta += 6.28318531f; }

			_progress += delta * 57.2957795f;
			_last_phase = in.phase;

			if (!std::isfinite(_progress) || !std::isfinite(in.roll_rate) || _progress < -10.f || in.roll_rate < -30.f) {
				fail("wrong roll direction");

			} else if (in.now - _started >= durationUs(settings.drive_timeout)) {
				fail("drive timeout");

			} else if (in.now - _started >= durationUs(settings.no_progress_timeout) && _progress < 5.f) {
				fail("no roll progress");

			} else if (!in.backend_ready || in.backend_session != _session) {
				fail("backend stopped drive");

			} else if (in.tilt <= settings.release_tilt || in.roll_rate >= 250.f
				   || (_progress >= 60.f && in.tilt - fmaxf(0.f, in.roll_rate) * 0.15f <= settings.release_tilt)) {
				_state = State::Coast;
				_started = in.now;
				_settled_since = 0;
				_throttle = 0.f;
				_reason = "coasting and restoring direction";

			} else {
				const float rate = fmaxf(30.f, settings.target_rate);
				const float change = in.roll_rate > rate ? -0.5f : settings.throttle_slew * (1.f - fmaxf(0.f, in.roll_rate) / rate);
				_throttle = fmaxf(0.f, fminf(fminf(settings.max_throttle, HARD_MAX_THROTTLE), _throttle + change * dt));
			}

		} else if (_state == State::Coast) {
			if (in.tilt < 25.f && in.gyro_norm < 10.f) {
				if (_settled_since == 0) { _settled_since = in.now; }

			} else { _settled_since = 0; }

			if (_settled_since != 0 && in.now - _settled_since >= 500000 && restored(in)) {
				_state = State::Done;
				_reason = "upright, stopped and direction restored";

			} else if (in.now - _started > 4000000) { fail("did not settle upright"); }

		} else if (_state == State::Cleanup && restored(in)) {
			_state = State::Failed;
		}

		return request(in.now);
	}

	void fail(const char *reason) { _state = State::Cleanup; _throttle = 0.f; _reason = reason; }
	bool active() const { return _state != State::Idle && _state != State::Done && _state != State::Failed; }
	State state() const { return _state; }
	const char *reason() const { return _reason; }
	float progress() const { return _progress; }
	float throttle() const { return _throttle; }
	uint64_t session() const { return _session; }
	Request request(uint64_t now) const
	{
		Request result{};
		result.timestamp = now;
		result.session = _session;
		result.action = _state == State::Prepare ? Action::Prepare : (_state == State::Drive ? Action::Drive : Action::Stop);
		result.throttle = _state == State::Drive ? _throttle : 0.f;
		return result;
	}

private:
	static uint64_t durationUs(float seconds)
	{
		// Only called after validTiming(). Round float seconds to integer microseconds.
		return static_cast<uint64_t>(static_cast<double>(seconds) * 1000000.0 + 0.5);
	}

	bool restored(const Input &in) const
	{
		return in.feedback_fresh && in.backend_idle && in.backend_clean && (_session == 0 || in.backend_session == _session);
	}
	State _state{State::Idle};
	bool _off_seen{false};
	uint64_t _session{0}, _started{0}, _last_now{0}, _settled_since{0};
	float _last_phase{0.f}, _progress{0.f}, _throttle{0.f};
	const char *_reason{"waiting for RC LOW then HIGH"};
};
} // namespace turtle_recovery
