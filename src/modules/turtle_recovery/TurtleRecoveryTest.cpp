// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team. All rights reserved.
#include <turtle_recovery/TurtleOutputGuard.hpp>
#include <turtle_recovery/TurtleFlipController.hpp>
#include <gtest/gtest.h>

using namespace turtle_recovery;

struct OutputFixture {
	TurtleOutputGuard guard;
	uint64_t now{1000000};
	Request request{1000000, 42, Action::Prepare, 0.f};
	OutputPermission permission{true, true, true, true, true, 0x9, 0xf, 0.12f};
	OutputFrame tick(uint64_t dt = 20000, bool renew = true)
	{
		now += dt;

		if (renew) { request.timestamp = now; }

		return guard.update(now, request, permission);
	}
	void ready()
	{
		EXPECT_EQ(tick().kind, FrameKind::Stop);

		for (unsigned i = 0; i < 10; ++i) {
			const auto frame = tick(i == 0 ? 500000 : 20000);
			EXPECT_EQ(frame.kind, FrameKind::Reverse);
			guard.frameSent(frame.kind);
		}

		EXPECT_EQ(guard.state(), OutputState::Ready);
		EXPECT_TRUE(guard.dirty());
	}
	void restored()
	{
		for (unsigned i = 0; i < 10; ++i) {
			const auto frame = tick(i == 0 ? 500000 : 20000, false);
			EXPECT_EQ(frame.kind, FrameKind::Restore);
			guard.frameSent(frame.kind);
		}

		EXPECT_FALSE(guard.busy());
		EXPECT_FALSE(guard.dirty());
	}
};

TEST(TurtleOutputGuard, RejectsMissingLeaseAndUprightEntry)
{
	OutputFixture f;
	f.request.timestamp = 0;
	EXPECT_EQ(f.tick(20000, false).kind, FrameKind::None);
	f.permission.entry = false;
	EXPECT_EQ(f.tick().kind, FrameKind::None);
	EXPECT_FALSE(f.guard.busy());
}

TEST(TurtleOutputGuard, OnlySelectedPinsReceiveBoundedThrottle)
{
	OutputFixture f;
	f.ready();
	f.request.action = Action::Drive;
	f.request.throttle = 1.f;
	auto frame = f.tick();
	EXPECT_EQ(frame.kind, FrameKind::Throttle);
	EXPECT_EQ(frame.mask, 0x9);
	EXPECT_FLOAT_EQ(frame.throttle, 0.12f);
	f.permission.throttle_limit = 0.4f;
	EXPECT_FLOAT_EQ(f.tick().throttle, 0.4f);
	f.permission.throttle_limit = 1.f;
	EXPECT_FLOAT_EQ(f.tick().throttle, 0.8f);
}

TEST(TurtleOutputGuard, ControllerDeathStopsAndRestoresWithoutHeartbeat)
{
	OutputFixture f;
	f.ready();
	f.request.action = Action::Drive;
	f.request.throttle = 0.1f;
	ASSERT_EQ(f.tick().kind, FrameKind::Throttle);
	EXPECT_EQ(f.tick(LEASE_US, false).kind, FrameKind::Stop);
	EXPECT_EQ(f.guard.state(), OutputState::StopRestore);
	f.permission.enabled = false;
	f.permission.live = false;
	f.restored();
}

TEST(TurtleOutputGuard, KillStopsThrustAndDefersDirectionCommands)
{
	OutputFixture f;
	f.ready();
	f.request.action = Action::Drive;
	f.request.throttle = 0.1f;
	f.tick();
	f.permission.stop_safe = false;
	EXPECT_EQ(f.tick().kind, FrameKind::Stop);
	EXPECT_EQ(f.tick(1000000).kind, FrameKind::Stop);
	EXPECT_TRUE(f.guard.dirty());
	f.permission.stop_safe = true;
	EXPECT_EQ(f.tick().kind, FrameKind::Stop);
	f.restored();
}

TEST(TurtleOutputGuard, PartialReverseIsDirtyAndRestored)
{
	OutputFixture f;
	f.tick();
	auto frame = f.tick(500000);
	f.guard.frameSent(frame.kind);
	EXPECT_TRUE(f.guard.dirty());
	f.request.action = Action::Stop;
	EXPECT_EQ(f.tick().kind, FrameKind::Stop);
	f.restored();
}

TEST(TurtleOutputGuard, MappingChangeRestoresOriginalPins)
{
	OutputFixture f;
	f.ready();
	f.permission.right_mask = 0x6;
	EXPECT_EQ(f.tick().kind, FrameKind::Stop);
	const auto frame = f.tick(500000);
	EXPECT_EQ(frame.kind, FrameKind::Restore);
	EXPECT_EQ(frame.mask, 0x9);
}

TEST(TurtleOutputGuard, StartupRestoresWhileRcIsUnavailable)
{
	OutputFixture f;
	f.permission.enabled = false;
	f.permission.live = false;
	f.request.timestamp = 0;
	f.guard.restore(f.now, 0xf);
	EXPECT_EQ(f.tick(20000, false).kind, FrameKind::Stop);
	f.restored();
}

TEST(TurtleOutputGuard, CompletedSessionCannotRestart)
{
	OutputFixture f;
	f.ready();
	f.request.action = Action::Stop;
	f.tick();
	f.restored();
	f.request.action = Action::Prepare;
	EXPECT_EQ(f.tick().kind, FrameKind::None);
	f.request.session = 43;
	EXPECT_EQ(f.tick().kind, FrameKind::Stop);
}

TEST(TurtleOutputGuard, NaNThrottleStopsAndDriveHasHardTimeout)
{
	OutputFixture f;
	f.ready();
	f.request.action = Action::Drive;
	f.request.throttle = NAN;
	EXPECT_EQ(f.tick().kind, FrameKind::Stop);
	EXPECT_EQ(f.guard.state(), OutputState::StopRestore);
	OutputFixture g;
	g.ready();
	g.request.action = Action::Drive;
	g.request.throttle = 0.1f;
	g.tick();
	EXPECT_EQ(g.tick(3000000).kind, FrameKind::Stop);
}

struct FlipFixture {
	TurtleFlipController controller;
	TurtleFlipController::Input in{};
	TurtleFlipController::Settings settings{};
	FlipFixture()
	{
		in.now = 1000000;
		in.enabled = in.safe = in.entry = in.feedback_fresh = in.backend_idle = in.backend_clean = true;
		in.tilt = 180.f;
		in.phase = 3.14159265f;
	}
	Request tick(uint64_t dt = 20000) { in.now += dt; return controller.update(in, settings); }
	void drive()
	{
		in.low = true; tick(); in.low = false; in.high = true; tick();
		ASSERT_EQ(tick(500000).action, Action::Prepare);
		in.backend_session = controller.session();
		in.backend_idle = in.backend_clean = false;
		in.backend_ready = true;
		EXPECT_EQ(tick().action, Action::Drive);
	}
};

TEST(TurtleFlipController, HighAtStartupAndUprightCannotStart)
{
	FlipFixture f;
	f.in.high = true;
	EXPECT_EQ(f.tick().action, Action::Stop);
	EXPECT_EQ(f.controller.state(), TurtleFlipController::State::Idle);
	f.in.high = false; f.in.low = true; f.tick();
	f.in.low = false; f.in.high = true; f.in.entry = false;
	EXPECT_EQ(f.tick().action, Action::Stop);
	EXPECT_EQ(f.controller.state(), TurtleFlipController::State::Failed);
}

TEST(TurtleFlipController, PositiveRollWrapReleaseAndStableUprightCompletion)
{
	FlipFixture f;
	f.drive();
	f.in.phase = -3.14159265f + 0.174532925f;
	f.in.tilt = 170.f; f.in.roll_rate = 60.f;
	EXPECT_EQ(f.tick().action, Action::Drive);
	EXPECT_NEAR(f.controller.progress(), 10.f, 1e-3f);
	f.in.phase = -3.14159265f + 1.57079633f; f.in.tilt = 90.f;
	EXPECT_EQ(f.tick(200000).action, Action::Drive);
	f.in.phase = -3.14159265f + 2.18166156f; f.in.tilt = 55.f;
	EXPECT_EQ(f.tick(200000).action, Action::Stop);
	EXPECT_EQ(f.controller.state(), TurtleFlipController::State::Coast);
	f.in.tilt = 5.f; f.in.gyro_norm = 1.f; f.tick();
	f.in.backend_idle = f.in.backend_clean = true; f.in.backend_ready = false;
	f.tick(600000);
	EXPECT_EQ(f.controller.state(), TurtleFlipController::State::Done);
	EXPECT_EQ(f.tick().action, Action::Stop); // HIGH held: no repeat
}

TEST(TurtleFlipController, SwitchOffOrInvalidSensorStopsImmediately)
{
	FlipFixture f; f.drive();
	f.in.safe = false;
	EXPECT_EQ(f.tick().action, Action::Stop);
	EXPECT_EQ(f.controller.state(), TurtleFlipController::State::Cleanup);
	f.in.backend_idle = f.in.backend_clean = true;
	f.tick();
	EXPECT_EQ(f.controller.state(), TurtleFlipController::State::Failed);
	FlipFixture g; g.drive(); g.in.high = false;
	EXPECT_EQ(g.tick().action, Action::Stop);
}

TEST(TurtleFlipController, WrongRollAndNoProgressAbort)
{
	FlipFixture f; f.drive(); f.in.roll_rate = -40.f;
	EXPECT_EQ(f.tick().action, Action::Stop);
	EXPECT_EQ(f.controller.state(), TurtleFlipController::State::Cleanup);
	FlipFixture g; g.drive();
	EXPECT_EQ(g.tick(1300000).action, Action::Stop);
	EXPECT_EQ(g.controller.state(), TurtleFlipController::State::Cleanup);
}

TEST(TurtleFlipController, DefaultNoProgressStopsAtExactMicrosecond)
{
	FlipFixture f; f.drive();
	EXPECT_EQ(f.tick(1199999).action, Action::Drive);
	EXPECT_EQ(f.tick(1).action, Action::Stop);
	EXPECT_STREQ(f.controller.reason(), "no roll progress");
}

TEST(TurtleFlipController, ExtendedNoProgressAllowsLongerDriveWithoutRemovingTimeout)
{
	FlipFixture f;
	f.settings.no_progress_timeout = 2.f;
	f.settings.drive_timeout = 2.8f;
	f.drive();
	EXPECT_EQ(f.tick(1200000).action, Action::Drive);
	EXPECT_EQ(f.tick(799999).action, Action::Drive);
	EXPECT_EQ(f.tick(1).action, Action::Stop);
	EXPECT_STREQ(f.controller.reason(), "no roll progress");
}

TEST(TurtleFlipController, ExtendedTotalTimeoutStopsEvenWhenRollHasProgress)
{
	FlipFixture f;
	f.settings.no_progress_timeout = 2.5f;
	f.settings.drive_timeout = 2.8f;
	f.drive();
	f.in.phase = -3.14159265f + 0.104719755f;
	f.in.tilt = 174.f;
	ASSERT_EQ(f.tick().action, Action::Drive);
	ASSERT_GT(f.controller.progress(), 5.f);
	EXPECT_EQ(f.tick(2779999).action, Action::Drive);
	EXPECT_EQ(f.tick(1).action, Action::Stop);
	EXPECT_STREQ(f.controller.reason(), "drive timeout");
}

TEST(TurtleFlipController, EqualTimeoutsReportTotalDriveTimeout)
{
	FlipFixture f;
	f.settings.no_progress_timeout = f.settings.drive_timeout = 2.5f;
	f.drive();
	EXPECT_EQ(f.tick(2499999).action, Action::Drive);
	EXPECT_EQ(f.tick(1).action, Action::Stop);
	EXPECT_STREQ(f.controller.reason(), "drive timeout");
}

TEST(TurtleFlipController, MinimumTimeoutIsSupported)
{
	FlipFixture f;
	f.settings.no_progress_timeout = f.settings.drive_timeout = 0.5f;
	f.drive();
	EXPECT_EQ(f.tick(499999).action, Action::Drive);
	EXPECT_EQ(f.tick(1).action, Action::Stop);
	EXPECT_STREQ(f.controller.reason(), "drive timeout");
}

TEST(TurtleFlipController, InvalidTimeSettingsCannotStart)
{
	const float times[][2] {
		{NAN, 2.5f}, {1.2f, NAN}, {INFINITY, 2.5f}, {1.2f, INFINITY},
		{0.49f, 2.5f}, {2.51f, 2.8f}, {0.5f, 0.49f}, {1.2f, 2.81f}, {2.f, 1.5f}
	};

	for (const auto &time : times) {
		FlipFixture f;
		f.settings.no_progress_timeout = time[0];
		f.settings.drive_timeout = time[1];
		f.in.low = true;
		f.tick();
		f.in.low = false;
		f.in.high = true;
		EXPECT_EQ(f.tick().action, Action::Stop);
		EXPECT_FALSE(f.controller.active());
		EXPECT_EQ(f.controller.session(), 0u);
		EXPECT_STREQ(f.controller.reason(), "invalid recovery time settings");
	}
}

TEST(TurtleFlipController, InvalidTimeDuringDriveStopsAndCleanupStillCompletes)
{
	FlipFixture f; f.drive();
	f.settings.drive_timeout = NAN;
	EXPECT_EQ(f.tick().action, Action::Stop);
	EXPECT_EQ(f.controller.state(), TurtleFlipController::State::Cleanup);
	EXPECT_STREQ(f.controller.reason(), "invalid recovery time settings");
	f.in.backend_idle = f.in.backend_clean = true;
	f.tick();
	EXPECT_EQ(f.controller.state(), TurtleFlipController::State::Failed);
}

TEST(TurtleFlipController, ExtendedTimeoutDoesNotDelaySafetyStops)
{
	for (unsigned condition = 0; condition < 3; ++condition) {
		FlipFixture f;
		f.settings.no_progress_timeout = 2.f;
		f.settings.drive_timeout = 2.8f;
		f.drive();

		if (condition == 0) { f.in.high = false; }

		if (condition == 1) { f.in.safe = false; }

		if (condition == 2) { f.in.feedback_fresh = false; }

		EXPECT_EQ(f.tick().action, Action::Stop);
		EXPECT_EQ(f.controller.state(), TurtleFlipController::State::Cleanup);
	}
}

TEST(TurtleFlipController, ConfiguredSlewCanReachEightTenthsWithoutOvershoot)
{
	FlipFixture f;
	f.settings.max_throttle = 0.8f;
	f.settings.throttle_slew = 1.f;
	f.settings.no_progress_timeout = 2.f;
	f.settings.drive_timeout = 2.8f;
	f.drive();
	float peak = 0.f;

	for (unsigned i = 0; i < 50; ++i) {
		const auto request = f.tick();
		ASSERT_EQ(request.action, Action::Drive);
		EXPECT_LE(request.throttle, 0.8f);
		peak = fmaxf(peak, request.throttle);
	}

	EXPECT_FLOAT_EQ(peak, 0.8f);
	EXPECT_FLOAT_EQ(f.controller.throttle(), 0.8f);
	f.in.high = false;
	EXPECT_EQ(f.tick().action, Action::Stop);
	EXPECT_FLOAT_EQ(f.controller.throttle(), 0.f);
}

TEST(TurtleFlipController, DefaultSlewCannotReachEightTenthsBeforeTotalTimeout)
{
	FlipFixture f;
	f.settings.max_throttle = 0.8f;
	f.settings.no_progress_timeout = 2.5f;
	f.settings.drive_timeout = 2.8f;
	f.drive();
	f.in.phase = -3.14159265f + 0.104719755f;
	f.in.tilt = 174.f; // progress prevents the no-progress timeout, isolating total duration
	float peak = 0.f;

	for (unsigned i = 0; i < 140; ++i) {
		const auto request = f.tick();
		peak = fmaxf(peak, request.throttle);
	}

	EXPECT_NEAR(peak, 0.695f, 1e-4f);
	EXPECT_LT(peak, 0.8f);
	EXPECT_EQ(f.controller.state(), TurtleFlipController::State::Cleanup);
	EXPECT_STREQ(f.controller.reason(), "drive timeout");
}

TEST(TurtleFlipController, FasterSlewStillRespectsRollRateFeedback)
{
	FlipFixture f;
	f.settings.max_throttle = 0.8f;
	f.settings.throttle_slew = 1.f;
	f.drive();
	EXPECT_NEAR(f.tick().throttle, 0.02f, 1e-6f);
	f.in.roll_rate = 120.f;
	EXPECT_NEAR(f.tick().throttle, 0.02f, 1e-6f);
	f.in.roll_rate = 180.f;
	EXPECT_NEAR(f.tick().throttle, 0.01f, 1e-6f); // fixed 0.5/s decrease is retained
}

TEST(TurtleFlipController, MinimumSlewIsSupported)
{
	FlipFixture f;
	f.settings.throttle_slew = 0.1f;
	f.drive();
	EXPECT_NEAR(f.tick().throttle, 0.002f, 1e-6f);
}

TEST(TurtleFlipController, InvalidSlewCannotStart)
{
	for (float slew : {NAN, INFINITY, -1.f, 0.f, 0.09f, 1.01f}) {
		FlipFixture f;
		f.settings.throttle_slew = slew;
		f.in.low = true; f.tick(); f.in.low = false; f.in.high = true;
		EXPECT_EQ(f.tick().action, Action::Stop);
		EXPECT_FALSE(f.controller.active());
		EXPECT_STREQ(f.controller.reason(), "invalid recovery throttle slew");
	}
}

TEST(TurtleFlipController, InvalidSlewDuringDriveStopsAndCleansUp)
{
	FlipFixture f; f.drive();
	f.settings.throttle_slew = NAN;
	EXPECT_EQ(f.tick().action, Action::Stop);
	EXPECT_EQ(f.controller.state(), TurtleFlipController::State::Cleanup);
	EXPECT_STREQ(f.controller.reason(), "invalid recovery throttle slew");
	f.in.backend_idle = f.in.backend_clean = true;
	f.tick();
	EXPECT_EQ(f.controller.state(), TurtleFlipController::State::Failed);
}

TEST(TurtleFlipController, MovementDuringReverseCancelsBeforeAnyDrive)
{
	FlipFixture f;
	f.in.low = true; f.tick(); f.in.low = false; f.in.high = true; f.tick();
	ASSERT_EQ(f.tick(500000).action, Action::Prepare);
	f.in.backend_session = f.controller.session();
	f.in.backend_ready = true;
	f.in.entry = false;
	EXPECT_EQ(f.tick().action, Action::Stop);
	EXPECT_EQ(f.controller.state(), TurtleFlipController::State::Cleanup);
}

TEST(TurtleFlipController, RejectedRetryAfterPreviousSessionDoesNotWaitForWrongFeedback)
{
	FlipFixture f; f.drive();
	f.in.high = false;
	f.tick();
	f.in.backend_idle = f.in.backend_clean = true;
	f.tick();
	ASSERT_EQ(f.controller.state(), TurtleFlipController::State::Failed);
	f.in.low = true; f.tick(); f.in.low = false; f.in.high = true; f.tick();
	ASSERT_EQ(f.controller.state(), TurtleFlipController::State::Qualify);
	f.in.high = false;
	f.tick();
	EXPECT_EQ(f.controller.state(), TurtleFlipController::State::Failed);
}

TEST(TurtleRecoveryIntegration, RightRollRestoresAndHeldHighCannotRestart)
{
	TurtleFlipController controller;
	TurtleOutputGuard guard;
	TurtleFlipController::Input in{};
	TurtleFlipController::Settings settings{};
	OutputPermission permission{true, true, true, true, true, 0x9, 0xf, 0.12f};
	uint64_t now = 1000000;
	guard.restore(now, 0xf);
	Request request{};
	unsigned reverse_frames = 0, restore_frames = 0, drive_frames = 0;
	float roll_progress = 0.f;
	bool powered = false;

	for (unsigned step = 0; step < 1100; ++step) {
		now += 5000;
		in.now = now;
		in.enabled = in.safe = in.feedback_fresh = true;
		in.low = step < 200;
		in.high = !in.low;
		in.backend_session = guard.session();
		in.backend_idle = !guard.busy();
		in.backend_clean = !guard.dirty();
		in.backend_ready = guard.state() == OutputState::Ready || guard.state() == OutputState::Driving;
		in.entry = roll_progress < 1.f;
		in.tilt = fmaxf(0.f, 180.f - roll_progress);
		in.phase = 3.14159265f + roll_progress * 0.0174532925f;
		in.roll_rate = powered ? 120.f : 0.f;
		in.gyro_norm = in.roll_rate;

		if (step % 4 == 0) { request = controller.update(in, settings); }

		const auto frame = guard.update(now, request, permission);

		if (frame.kind == FrameKind::Reverse) { ++reverse_frames; EXPECT_EQ(frame.mask, 0x9); }

		if (frame.kind == FrameKind::Restore) { ++restore_frames; }

		if (frame.kind == FrameKind::Throttle && frame.throttle > 0.f) {
			++drive_frames;
			powered = true;
			EXPECT_EQ(frame.mask, 0x9);
			EXPECT_LE(frame.throttle, settings.max_throttle);
		}

		guard.frameSent(frame.kind);

		// Simulate inertia completing a positive roll after thrust release.
		if (powered) { roll_progress = fminf(180.f, roll_progress + 0.6f); }

		if (roll_progress >= 180.f) { powered = false; }
	}

	EXPECT_GT(drive_frames, 0u);
	EXPECT_EQ(reverse_frames, 10u);
	EXPECT_EQ(restore_frames, 20u); // startup all four, then the recovery pair
	EXPECT_FALSE(guard.busy());
	EXPECT_FALSE(guard.dirty());
	EXPECT_EQ(controller.state(), TurtleFlipController::State::Done);
}
