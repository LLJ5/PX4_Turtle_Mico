// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 PX4 Development Team. All rights reserved.

#include "DShotTurtleCommand.hpp"
#include <gtest/gtest.h>

using State = DShotTurtleCommand::State;

TEST(DShotTurtleCommand, WaitsForStopAndCountsOnlySentFrames)
{
	DShotTurtleCommand command;
	command.start(1000000);
	EXPECT_FALSE(command.update(1000000, true, true, true));
	command.frameSent(); // Waiting cycles must not consume repetitions.
	EXPECT_FALSE(command.update(1499999, true, true, true));
	ASSERT_TRUE(command.update(1500000, true, true, true));

	for (unsigned i = 0; i < 9; ++i) {
		command.frameSent();
		EXPECT_FALSE(command.finished());
	}

	command.frameSent();
	EXPECT_EQ(command.state(), State::Sent);
	EXPECT_TRUE(command.finished());
	EXPECT_FALSE(command.update(1600000, true, true, true));
}

TEST(DShotTurtleCommand, MovingOutputsRestartStopWait)
{
	DShotTurtleCommand command;
	command.start(1000000);
	EXPECT_FALSE(command.update(1000000, true, true, true));
	EXPECT_FALSE(command.update(1400000, true, true, false));
	EXPECT_FALSE(command.update(1500000, true, true, true));
	EXPECT_FALSE(command.update(1999999, true, true, true));
	EXPECT_TRUE(command.update(2000000, true, true, true));
}

TEST(DShotTurtleCommand, ArmingOrInvalidModeAborts)
{
	DShotTurtleCommand command;
	command.start(1000000);
	EXPECT_FALSE(command.update(1000000, false, true, true));
	EXPECT_EQ(command.state(), State::Aborted);
	EXPECT_FALSE(command.update(2000000, true, true, true));
}

TEST(DShotTurtleCommand, InvalidOrChangedTargetAborts)
{
	DShotTurtleCommand command;
	command.start(1000000);
	EXPECT_FALSE(command.update(1000000, true, false, true));
	EXPECT_EQ(command.state(), State::Aborted);
	command.start(2000000);
	command.update(2000000, true, true, true);
	ASSERT_TRUE(command.update(2500000, true, true, true));
	command.frameSent();
	EXPECT_FALSE(command.update(2510000, true, false, true));
	EXPECT_EQ(command.state(), State::Aborted);
}

TEST(DShotTurtleCommand, ThrottleDuringSendingAborts)
{
	DShotTurtleCommand command;
	command.start(1000000);
	command.update(1000000, true, true, true);
	ASSERT_TRUE(command.update(1500000, true, true, true));
	EXPECT_FALSE(command.update(1510000, true, true, false));
	EXPECT_EQ(command.state(), State::Aborted);
}

TEST(DShotTurtleCommand, TimeoutsAndFreshRequest)
{
	DShotTurtleCommand command;
	command.start(1000000);
	EXPECT_FALSE(command.update(4000000, true, true, true));
	EXPECT_EQ(command.state(), State::TimedOut);
	command.start(5000000);
	EXPECT_FALSE(command.update(5000000, true, true, true));
	EXPECT_TRUE(command.update(5500000, true, true, true));
	EXPECT_EQ(command.state(), State::Sending);
}

TEST(DShotTurtleCommand, InactiveAndInvalidTimeCannotSend)
{
	DShotTurtleCommand command;
	EXPECT_FALSE(command.update(1000000, true, true, true));
	command.start(1000000);
	EXPECT_FALSE(command.update(999999, true, true, true));
	EXPECT_EQ(command.state(), State::Aborted);
}
