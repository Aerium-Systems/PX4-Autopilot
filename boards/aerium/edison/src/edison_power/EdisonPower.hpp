/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

/**
 * @file EdisonPower.hpp
 *
 * Aerium Edison power board: averages the board ADC inputs, reads the LM5143
 * PG / HOST_5V status pins, drives the LM5143 enables from parameters and
 * publishes everything on DroneCAN through the uavcannode node.
 *
 * Runs on the uavcan work queue, the same thread as UavcanNode::Run(), so the
 * libuavcan node is never used from two threads at once.
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/adc_report.h>
#include <uORB/topics/parameter_update.h>

#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/device/Temperature.hpp>
#include <uavcan/equipment/power/BatteryInfo.hpp>
#include <uavcan/equipment/power/CircuitStatus.hpp>

class EdisonPower : public ModuleBase<EdisonPower>, public ModuleParams, public px4::WorkItem
{
public:
	EdisonPower();
	~EdisonPower() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	enum AdcInput : uint8_t {
		VIN = 0,   // V_SENSE    (input voltage)
		IIN,       // C_SENSE    (input current)
		VCC_A,     // VCCA_SENSE (output rail A)
		VCC_B,     // VCCB_SENSE (output rail B)
		TEMP1,     // TEMP1      (NTC TH2)
		TEMP2,     // TEMP2      (NTC TH1)
		ADC_INPUT_COUNT
	};

	struct Measurements {
		float vin_v{NAN};
		float iin_a{NAN};
		float vcc_a_v{NAN};
		float vcc_b_v{NAN};
		float temp1_c{NAN};
		float temp2_c{NAN};
		bool en_a{false};
		bool en_b{false};
		bool pg_a{false};
		bool pg_b{false};
		bool host_5v{false};
	};

	// NodeStatus.vendor_specific_status_code bits
	static constexpr uint16_t STATUS_EN_A    = 1 << 0;
	static constexpr uint16_t STATUS_PG_A    = 1 << 1;
	static constexpr uint16_t STATUS_EN_B    = 1 << 2;
	static constexpr uint16_t STATUS_PG_B    = 1 << 3;
	static constexpr uint16_t STATUS_HOST_5V = 1 << 4;

	void Run() override;

	void applyRailEnables();
	void accumulate(const adc_report_s &adc);
	Measurements computeAndReset();
	float ntcToCelsius(float adc_ratio) const;
	bool ensurePublishers();
	void publish(const Measurements &m);

	uORB::SubscriptionCallbackWorkItem _adc_report_sub{this, ORB_ID(adc_report)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	// Sums of raw ADC counts since the last publish
	float _adc_sum[ADC_INPUT_COUNT] {};
	uint32_t _adc_count{0};
	uint32_t _adc_resolution{4096};
	float _adc_v_ref{3.3f};

	hrt_abstime _last_publish{0};
	Measurements _last{};

	// DroneCAN publishers on uavcannode's node, created once that node has a node ID
	const void *_node_owner{nullptr};
	uavcan::Publisher<uavcan::equipment::power::BatteryInfo> *_pub_battery{nullptr};
	uavcan::Publisher<uavcan::equipment::power::CircuitStatus> *_pub_circuit{nullptr};
	uavcan::Publisher<uavcan::equipment::device::Temperature> *_pub_temperature{nullptr};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _publish_perf{perf_alloc(PC_COUNT, MODULE_NAME": can publish")};
	perf_counter_t _publish_err_perf{perf_alloc(PC_COUNT, MODULE_NAME": can publish error")};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::EDSN_EN_A>) _param_en_a,
		(ParamBool<px4::params::EDSN_EN_B>) _param_en_b,
		(ParamFloat<px4::params::EDSN_RATE>) _param_rate,
		(ParamFloat<px4::params::EDSN_V_DIV>) _param_v_div,
		(ParamFloat<px4::params::EDSN_A_PER_V>) _param_a_per_v,
		(ParamFloat<px4::params::EDSN_RAIL_DIV>) _param_rail_div,
		(ParamFloat<px4::params::EDSN_NTC_B>) _param_ntc_b,
		(ParamFloat<px4::params::EDSN_T_HOT>) _param_t_hot,
		(ParamInt<px4::params::EDSN_ID_BASE>) _param_id_base
	)
};
