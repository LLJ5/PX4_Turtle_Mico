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
//基础逻辑测试
#include "TurtleObserver.hpp"
#include <gtest/gtest.h>

using namespace turtle_recovery;

TEST(TurtleObserver, SwitchPositionUsesCurrentLevel)
{
	EXPECT_EQ(switchPosition(true, 1.f), SwitchPosition::High);
	EXPECT_EQ(switchPosition(true, -1.f), SwitchPosition::Low);
	EXPECT_EQ(switchPosition(true, 0.f), SwitchPosition::Middle);
	EXPECT_EQ(switchPosition(true, 0.5f), SwitchPosition::Middle);
	EXPECT_EQ(switchPosition(true, -0.5f), SwitchPosition::Middle);
	EXPECT_EQ(switchPosition(false, 1.f), SwitchPosition::Unknown);
	EXPECT_EQ(switchPosition(true, NAN), SwitchPosition::Unknown);
	EXPECT_EQ(switchPosition(true, 2.f), SwitchPosition::Unknown);

	ObserveTrigger trigger;
	EXPECT_FALSE(trigger.update(true, 1.f));
	EXPECT_EQ(switchPosition(true, 1.f), SwitchPosition::High);
}

TEST(TurtleObserver, UnchangedRcRefreshDoesNotResetOnSwitch)
{
	ObserveTrigger trigger;
	uint64_t sample = 1000000;
	EXPECT_FALSE(trigger.update(rcSampleFresh(sample, sample, 0.5f), -1.f));
	ASSERT_TRUE(trigger.update(rcSampleFresh(sample, sample, 0.5f), 1.f));

	for (uint64_t now = sample + 20000; now < 3000000; now += 20000) {
		if (now - sample >= 320000) {
			sample = now;
		}

		ASSERT_TRUE(rcSampleFresh(now, sample, 0.5f));
		EXPECT_FALSE(trigger.update(rcSampleFresh(now, sample, 0.5f), 1.f));
		EXPECT_TRUE(trigger.active());
		EXPECT_TRUE(trigger.seenOff());
	}

	EXPECT_FALSE(trigger.update(rcSampleFresh(sample + 500000, sample, 0.5f), 1.f));
	EXPECT_FALSE(trigger.active());
	EXPECT_FALSE(trigger.update(true, 1.f));
	EXPECT_FALSE(trigger.update(true, -1.f));
	EXPECT_TRUE(trigger.update(true, 1.f));
}

TEST(TurtleObserver, RcFreshnessUsesSampleTimeAndConfiguredTimeout)
{
	EXPECT_TRUE(rcSampleFresh(1400000, 1000000, 0.5f));
	EXPECT_FALSE(rcSampleFresh(1400000, 1000000, 0.3f));
	EXPECT_FALSE(rcSampleFresh(1500000, 1000000, 0.5f));
	EXPECT_FALSE(rcSampleFresh(1000000, 0, 0.5f));
	EXPECT_FALSE(rcSampleFresh(1000000, 1000001, 0.5f));
	EXPECT_FALSE(rcSampleFresh(1000000, 1000000, NAN));
	EXPECT_FALSE(rcSampleFresh(1000000, 1000000, 0.f));
}

TEST(TurtleObserver, AttitudeAcrossFullRoll)
{
	for (int degrees = 0; degrees <= 360; ++degrees) {
		const float angle = degrees * 0.017453292519943295f;
		const float q[4] {cosf(angle / 2.f), sinf(angle / 2.f), 0.f, 0.f};
		const float negative_q[4] {-q[0], -q[1], -q[2], -q[3]};
		float alignment;
		float negative_alignment;
		ASSERT_TRUE(attitudeAlignment(q, alignment));
		ASSERT_TRUE(attitudeAlignment(negative_q, negative_alignment));
		EXPECT_NEAR(alignment, cosf(angle), 1e-5f);
		EXPECT_FLOAT_EQ(alignment, negative_alignment);
	}
}

TEST(TurtleObserver, PitchAtEulerSingularity)
{
	const float q[4] {sqrtf(0.5f), 0.f, sqrtf(0.5f), 0.f};
	float alignment;
	ASSERT_TRUE(attitudeAlignment(q, alignment));
	EXPECT_NEAR(alignment, 0.f, 1e-5f);
}

TEST(TurtleObserver, InvalidAttitude)
{
	const float zero[4] {};
	const float invalid[4] {NAN, 0.f, 0.f, 0.f};
	const float scaled[4] {2.f, 0.f, 0.f, 0.f};
	float alignment;
	EXPECT_FALSE(attitudeAlignment(zero, alignment));
	EXPECT_FALSE(attitudeAlignment(invalid, alignment));
	EXPECT_FALSE(attitudeAlignment(scaled, alignment));
	EXPECT_TRUE(std::isnan(alignment));
}

TEST(TurtleObserver, RightMotorsFollowGeometryInsteadOfMotorNumbers)
{
	const float px4_x[4] {1.f, -1.f, -1.f, 1.f};
	const float remapped[4] {-1.f, 1.f, 1.f, -1.f};
	const float plus[4] {0.f, -1.f, 1.f, 0.f};
	const float invalid[4] {NAN, -1.f, -1.f, 1.f};
	const float asymmetric[4] {1.f, 1.f, 1.f, -1.f};
	uint32_t mask;
	ASSERT_TRUE(rightMotorMask(px4_x, mask));
	EXPECT_EQ(mask, 0x9u);
	ASSERT_TRUE(rightMotorMask(remapped, mask));
	EXPECT_EQ(mask, 0x6u);
	EXPECT_FALSE(rightMotorMask(plus, mask));
	EXPECT_EQ(mask, 0u);
	EXPECT_FALSE(rightMotorMask(invalid, mask));
	EXPECT_EQ(mask, 0u);
	EXPECT_FALSE(rightMotorMask(asymmetric, mask));
	EXPECT_EQ(mask, 0u);
}

TEST(TurtleObserver, StartsOnAndHeldOnDoNotRetrigger)
{
	ObserveTrigger trigger;
	EXPECT_FALSE(trigger.update(true, 1.f));
	EXPECT_FALSE(trigger.active());
	EXPECT_FALSE(trigger.update(true, -1.f));
	EXPECT_TRUE(trigger.update(true, 1.f));
	EXPECT_TRUE(trigger.active());

	for (int i = 0; i < 100; ++i) {
		EXPECT_FALSE(trigger.update(true, 1.f));
	}

	EXPECT_FALSE(trigger.update(true, -1.f));
	EXPECT_FALSE(trigger.active());
	EXPECT_TRUE(trigger.update(true, 1.f));
}

TEST(TurtleObserver, InvalidInputAndUnavailableRequireOffAgain)
{
	ObserveTrigger trigger;
	trigger.update(true, -1.f);
	ASSERT_TRUE(trigger.update(true, 1.f));
	EXPECT_FALSE(trigger.update(false, 1.f));
	EXPECT_FALSE(trigger.active());
	EXPECT_FALSE(trigger.update(true, 1.f));
	trigger.update(true, -1.f);
	EXPECT_FALSE(trigger.update(true, NAN));
	EXPECT_FALSE(trigger.update(true, 1.f));
	trigger.update(true, -1.f);
	EXPECT_TRUE(trigger.update(true, 1.f));
}
