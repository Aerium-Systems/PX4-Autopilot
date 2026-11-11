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

#include "EdisonPower.hpp"

#include <board_config.h>
#include <px4_platform_common/micro_hal.h>
#include <lib/mathlib/mathlib.h>

#include <drivers/uavcannode/UavcanNode.hpp>

#include <math.h>

namespace
{

// Must stay in the order of EdisonPower::AdcInput
constexpr int16_t kAdcChannels[] = {
	ADC_BATTERY_VOLTAGE_CHANNEL, // VIN
	ADC_BATTERY_CURRENT_CHANNEL, // IIN
	ADC_VCC_A_SENSE_CHANNEL,     // VCC_A
	ADC_VCC_B_SENSE_CHANNEL,     // VCC_B
	ADC_TEMP1_CHANNEL,           // TEMP1
	ADC_TEMP2_CHANNEL,           // TEMP2
};

// TEMP_SENSE sheet: 3.3V -> 12k (R30/R45) -> TEMPx -> NTC 10k@25C (TH1/TH2) -> AGND.
// The divider and the ADC (VDDA) share the 3.3V LDO, so the reading is ratiometric.
constexpr float kNtcPullupOhm = 12000.f;
constexpr float kNtcR25Ohm = 10000.f;
constexpr float kKelvinAt0C = 273.15f;
constexpr float kNtcT25Kelvin = 25.f + kKelvinAt0C;

constexpr char kModelName[] = "Aerium Edison";

} // namespace

EdisonPower::EdisonPower() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::uavcan)
{
	static_assert(sizeof(kAdcChannels) / sizeof(kAdcChannels[0]) == ADC_INPUT_COUNT, "ADC channel table size");
}

EdisonPower::~EdisonPower()
{
	// Only delete the publishers while the node they reference still exists.
	if ((_node_owner != nullptr) && (_node_owner == uavcannode::UavcanNode::instance())) {
		delete _pub_battery;
		delete _pub_circuit;
		delete _pub_temperature;
	}

	perf_free(_cycle_perf);
	perf_free(_publish_perf);
	perf_free(_publish_err_perf);
}

bool EdisonPower::init()
{
	applyRailEnables();

	if (!_adc_report_sub.registerCallback()) {
		PX4_ERR("adc_report callback registration failed");
		return false;
	}

	return true;
}

void EdisonPower::applyRailEnables()
{
	px4_arch_gpiowrite(GPIO_EN_VCC_A, _param_en_a.get());
	px4_arch_gpiowrite(GPIO_EN_VCC_B, _param_en_b.get());
}

void EdisonPower::Run()
{
	if (should_exit()) {
		_adc_report_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
		applyRailEnables();
	}

	adc_report_s adc_report;

	if (_adc_report_sub.update(&adc_report)) {
		accumulate(adc_report);
	}

	const float rate_hz = math::constrain(_param_rate.get(), 1.f, 50.f);
	const hrt_abstime now = hrt_absolute_time();

	if ((_adc_count > 0) && (now - _last_publish >= static_cast<hrt_abstime>(1e6f / rate_hz))) {
		_last_publish = now;
		_last = computeAndReset();
		publish(_last);
	}

	perf_end(_cycle_perf);
}

void EdisonPower::accumulate(const adc_report_s &adc)
{
	float sample[ADC_INPUT_COUNT] {};
	bool found[ADC_INPUT_COUNT] {};

	for (unsigned i = 0; i < sizeof(adc.channel_id) / sizeof(adc.channel_id[0]); i++) {
		for (unsigned k = 0; k < ADC_INPUT_COUNT; k++) {
			if (adc.channel_id[i] == kAdcChannels[k]) {
				sample[k] = static_cast<float>(adc.raw_data[i]);
				found[k] = true;
			}
		}
	}

	for (unsigned k = 0; k < ADC_INPUT_COUNT; k++) {
		if (!found[k]) {
			return; // incomplete report, skip it
		}
	}

	for (unsigned k = 0; k < ADC_INPUT_COUNT; k++) {
		_adc_sum[k] += sample[k];
	}

	_adc_count++;

	if (adc.resolution > 0) {
		_adc_resolution = adc.resolution;
	}

	if (adc.v_ref > 0.f) {
		_adc_v_ref = adc.v_ref;
	}
}

EdisonPower::Measurements EdisonPower::computeAndReset()
{
	float avg[ADC_INPUT_COUNT];

	for (unsigned k = 0; k < ADC_INPUT_COUNT; k++) {
		avg[k] = _adc_sum[k] / static_cast<float>(_adc_count);
		_adc_sum[k] = 0.f;
	}

	_adc_count = 0;

	const float full_scale = static_cast<float>(_adc_resolution);
	const float volts_per_count = _adc_v_ref / full_scale;

	Measurements m{};
	m.vin_v   = avg[VIN] * volts_per_count * _param_v_div.get();
	m.iin_a   = avg[IIN] * volts_per_count * _param_a_per_v.get();
	m.vcc_a_v = avg[VCC_A] * volts_per_count * _param_rail_div.get();
	m.vcc_b_v = avg[VCC_B] * volts_per_count * _param_rail_div.get();
	m.temp1_c = ntcToCelsius(avg[TEMP1] / full_scale);
	m.temp2_c = ntcToCelsius(avg[TEMP2] / full_scale);

	m.en_a    = _param_en_a.get();
	m.en_b    = _param_en_b.get();
	m.pg_a    = px4_arch_gpioread(GPIO_PG_VCC_A);
	m.pg_b    = px4_arch_gpioread(GPIO_PG_VCC_B);
	m.host_5v = px4_arch_gpioread(GPIO_HOST_5V_VALID);

	return m;
}

float EdisonPower::ntcToCelsius(float adc_ratio) const
{
	// Near the rails the thermistor is open or shorted: report "unknown".
	if (!(adc_ratio > 0.005f && adc_ratio < 0.995f)) {
		return NAN;
	}

	const float r_ntc = kNtcPullupOhm * adc_ratio / (1.f - adc_ratio);
	const float inv_t = 1.f / kNtcT25Kelvin + logf(r_ntc / kNtcR25Ohm) / _param_ntc_b.get();

	return 1.f / inv_t - kKelvinAt0C;
}

bool EdisonPower::ensurePublishers()
{
	uavcannode::UavcanNode *const owner = uavcannode::UavcanNode::instance();

	if (owner != _node_owner) {
		// uavcannode stopped or was restarted. Publishers created on the old node
		// can't be destroyed safely once it's gone, so they are dropped (leaked);
		// this only happens on a manual "uavcannode stop".
		_pub_battery = nullptr;
		_pub_circuit = nullptr;
		_pub_temperature = nullptr;
		_node_owner = nullptr;
	}

	if (owner == nullptr) {
		return false;
	}

	uavcan::Node<> &node = owner->get_node();

	// Wait until the node runs with a node ID (from the bootloader or dynamic allocation).
	if (!node.isStarted() || !node.getNodeID().isUnicast()) {
		return false;
	}

	if (_node_owner == nullptr) {
		_pub_battery = new uavcan::Publisher<uavcan::equipment::power::BatteryInfo>(node);
		_pub_circuit = new uavcan::Publisher<uavcan::equipment::power::CircuitStatus>(node);
		_pub_temperature = new uavcan::Publisher<uavcan::equipment::device::Temperature>(node);

		if ((_pub_battery == nullptr) || (_pub_circuit == nullptr) || (_pub_temperature == nullptr)) {
			delete _pub_battery;
			delete _pub_circuit;
			delete _pub_temperature;
			_pub_battery = nullptr;
			_pub_circuit = nullptr;
			_pub_temperature = nullptr;
			PX4_ERR("publisher alloc failed");
			return false;
		}

		_pub_battery->setPriority(uavcan::TransferPriority::MiddleLower);
		_pub_circuit->setPriority(uavcan::TransferPriority::MiddleLower);
		_pub_temperature->setPriority(uavcan::TransferPriority::Lowest);

		_node_owner = owner;
	}

	return true;
}

void EdisonPower::publish(const Measurements &m)
{
	if (!ensurePublishers()) {
		return;
	}

	uavcan::Node<> &node = uavcannode::UavcanNode::instance()->get_node();
	const uint8_t id_base = static_cast<uint8_t>(math::constrain(_param_id_base.get(), (int32_t)0, (int32_t)254));
	const float t_hot = _param_t_hot.get();
	int errors = 0;

	// Input power (IP+/IP-) -> uavcan.equipment.power.BatteryInfo
	{
		uavcan::equipment::power::BatteryInfo msg{};
		msg.temperature = NAN;              // no battery temperature sensor on the board
		msg.voltage = m.vin_v;
		msg.current = m.iin_a;              // INA241 is unidirectional: discharge current >= 0
		msg.average_power_10sec = NAN;
		msg.remaining_capacity_wh = NAN;
		msg.full_charge_capacity_wh = NAN;
		msg.hours_to_full_charge = 0.f;
		msg.status_flags = uavcan::equipment::power::BatteryInfo::STATUS_FLAG_IN_USE;
		msg.state_of_health_pct = uavcan::equipment::power::BatteryInfo::STATE_OF_HEALTH_UNKNOWN;
		// No state-of-charge estimate on the board (use UAVCAN_SUB_BAT = 2 on the flight controller).
		msg.state_of_charge_pct = 127;
		msg.state_of_charge_pct_stdev = 0;
		msg.battery_id = id_base;
		msg.model_instance_id = 0;
		msg.model_name = kModelName;

		errors += (_pub_battery->broadcast(msg) < 0);
	}

	// Output rails -> uavcan.equipment.power.CircuitStatus (circuit_id = base + 0 / base + 1)
	{
		const struct {
			float voltage;
			bool enabled;
			bool power_good;
		} rails[2] = {
			{m.vcc_a_v, m.en_a, m.pg_a},
			{m.vcc_b_v, m.en_b, m.pg_b},
		};

		for (unsigned i = 0; i < 2; i++) {
			uavcan::equipment::power::CircuitStatus msg{};
			msg.circuit_id = id_base + i;
			msg.voltage = rails[i].voltage;
			msg.current = NAN; // per-rail current is not measured
			// LM5143 PG low while enabled = rail out of regulation
			msg.error_flags = (rails[i].enabled && !rails[i].power_good) ?
					  uavcan::equipment::power::CircuitStatus::ERROR_FLAG_UNDERVOLTAGE : 0;

			errors += (_pub_circuit->broadcast(msg) < 0);
		}
	}

	// Thermistors -> uavcan.equipment.device.Temperature (device_id = base + 0 / base + 1)
	bool overheating = false;

	{
		const float temps_c[2] = {m.temp1_c, m.temp2_c};

		for (unsigned i = 0; i < 2; i++) {
			const bool hot = PX4_ISFINITE(temps_c[i]) && (temps_c[i] > t_hot);
			overheating |= hot;

			uavcan::equipment::device::Temperature msg{};
			msg.device_id = id_base + i;
			msg.temperature = PX4_ISFINITE(temps_c[i]) ? (temps_c[i] + kKelvinAt0C) : NAN;
			msg.error_flags = hot ? uavcan::equipment::device::Temperature::ERROR_FLAG_OVERHEATING : 0;

			errors += (_pub_temperature->broadcast(msg) < 0);
		}
	}

	// EN / PG / HOST_5V -> NodeStatus vendor-specific status code, faults -> node health
	uint16_t status = 0;
	status |= m.en_a ? STATUS_EN_A : 0;
	status |= m.pg_a ? STATUS_PG_A : 0;
	status |= m.en_b ? STATUS_EN_B : 0;
	status |= m.pg_b ? STATUS_PG_B : 0;
	status |= m.host_5v ? STATUS_HOST_5V : 0;
	node.setVendorSpecificStatusCode(status);

	const bool rail_fault = (m.en_a && !m.pg_a) || (m.en_b && !m.pg_b);

	if (rail_fault || overheating) {
		node.setHealthWarning();

	} else {
		node.setHealthOk();
	}

	perf_count(_publish_perf);

	if (errors > 0) {
		perf_count(_publish_err_perf);
	}
}

int EdisonPower::print_status()
{
	PX4_INFO("input:   %.2f V  %.2f A", (double)_last.vin_v, (double)_last.iin_a);
	PX4_INFO("VCC_A:   %.2f V  EN %d  PG %d", (double)_last.vcc_a_v, _last.en_a, _last.pg_a);
	PX4_INFO("VCC_B:   %.2f V  EN %d  PG %d", (double)_last.vcc_b_v, _last.en_b, _last.pg_b);
	PX4_INFO("TEMP1:   %.1f C  TEMP2: %.1f C", (double)_last.temp1_c, (double)_last.temp2_c);
	PX4_INFO("HOST_5V: %s", _last.host_5v ? "present" : "absent");
	PX4_INFO("DroneCAN: %s", (_node_owner != nullptr) ? "publishing" : "waiting for uavcannode node ID");
	perf_print_counter(_cycle_perf);
	perf_print_counter(_publish_perf);
	perf_print_counter(_publish_err_perf);
	return 0;
}

int EdisonPower::task_spawn(int argc, char *argv[])
{
	EdisonPower *instance = new EdisonPower();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int EdisonPower::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int EdisonPower::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Aerium Edison power board monitor. Publishes on DroneCAN:
- uavcan.equipment.power.BatteryInfo: input voltage and current (battery_id = EDSN_ID_BASE)
- uavcan.equipment.power.CircuitStatus: VCC_A / VCC_B voltage, UNDERVOLTAGE when PG is low
- uavcan.equipment.device.Temperature: TEMP1 / TEMP2 NTCs, OVERHEATING above EDSN_T_HOT
- NodeStatus vendor-specific code: bit0 EN_A, bit1 PG_A, bit2 EN_B, bit3 PG_B, bit4 HOST_5V

Rails are switched with EDSN_EN_A / EDSN_EN_B (also settable over DroneCAN).
Requires board_adc and uavcannode.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("edison_power", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int edison_power_main(int argc, char *argv[])
{
	return EdisonPower::main(argc, argv);
}
