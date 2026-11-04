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
//倒扣判断、开关识别、遥控时效、右侧电机识别
#pragma once

#include <cmath>
#include <cstdint>
#include <matrix/matrix/math.hpp>
// 将这些函数和类放进专属命名空间，避免与其他模块同名冲突
namespace turtle_recovery
{
enum class SwitchPosition { Unknown, Low, Middle, High };

// 当前开关位置直接由 AUX 数值决定，不依赖是否曾经经过 OFF。
inline SwitchPosition switchPosition(bool valid, float aux)
{
	if (!valid || !std::isfinite(aux) || fabsf(aux) > 1.f) {
		return SwitchPosition::Unknown;
	}

	if (aux > 0.5f) {
		return SwitchPosition::High;
	}

	return aux < -0.5f ? SwitchPosition::Low : SwitchPosition::Middle;
}

// rc_update 在通道值不变时可能超过 300 ms 才更新，沿用 PX4 的失联超时。
inline bool rcSampleFresh(uint64_t now, uint64_t timestamp_sample, float timeout_s)
{
	return timestamp_sample != 0 && now >= timestamp_sample
	       && std::isfinite(timeout_s) && timeout_s > 0.f
	       && now - timestamp_sample < static_cast<double>(timeout_s) * 1e6;
}

// 判断机体 +Z 轴与地球向下方向的对齐程度，返回 true 表示四元数有效.


// Alignment of body +Z with earth down. No Euler angle conversion is needed.
inline bool attitudeAlignment(const float q[4], float &alignment)  //根据四元数判断机体倾斜程度
{//输入是姿态四元数 q，输出是 alignment，函数返回值表示输入姿态是否有效。
	alignment = NAN;
	matrix::Quatf attitude{q};//构造四元数对象
	const float norm = attitude.norm();//模长

	if (!std::isfinite(norm) || fabsf(norm - 1.f) > 0.02f) {
		return false;
	}//如果模长不是有限数，或与单位四元数的模长 1.0 相差超过 0.02，就返回 false。

	attitude.normalize(); //将四元数归一化，转换为方向余弦矩阵。
	alignment = matrix::Dcmf{attitude}(2, 2); //读取矩阵的 (2, 2) 元素，作为机体 +Z 轴与地球向下方向的对齐程度，返回 true
	return true;
}//用矩阵元素而不是欧拉角，能避免万向届思索


// Logical motor indices, not physical output channels. Only a conventional
// four-rotor layout with two motors on either side is supported in this stage.
inline bool rightMotorMask(const float position_y[4], uint32_t &mask)
{//position_y > 0 表示右侧电机，position_y < 0 表示左侧电机，mask 是输出的位掩码，表示右侧电机的索引。
	mask = 0;
	unsigned right_count = 0;
	unsigned left_count = 0;

	for (unsigned i = 0; i < 4; ++i) {
		if (!std::isfinite(position_y[i]) || fabsf(position_y[i]) < 1e-4f) {
			mask = 0;
			return false;
		}

		if (position_y[i] > 0.f) {
			mask |= 1u << i;
			++right_count;

		} else {
			++left_count;
		}
	}

	if (right_count != 2 || left_count != 2) {
		mask = 0;
		return false;
	}

	return true;
}

//检测辅助开关的 OFF→ON 触发
class ObserveTrigger
{
public:   //available：当前输入能不能信，aux：当前输入的开关
	bool update(bool available, float aux)
	{
		if (!available || !std::isfinite(aux) || aux < -1.f || aux > 1.f) {
			reset();
			return false;
		}

		if (aux < -0.5f) {
			_seen_off = true;
			_active = false;

		} else if (aux > 0.5f && _seen_off && !_active) {
			_active = true;
			return true;
		}

		return false;
	}
//reset()：把状态清零。_seen_off = false 表示还没观察到开关处于 OFF；
//_active = false 表示当前未激活。开关输入失效时，update() 会调用它。
	void reset() { _seen_off = false; _active = false; }
	bool active() const { return _active; }
	bool seenOff() const { return _seen_off; }

private:
	bool _seen_off{false};
	bool _active{false};
};

} // namespace turtle_recovery
