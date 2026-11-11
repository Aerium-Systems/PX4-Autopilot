/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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

#include <px4_arch/i2c_hw_description.h>

#include <drivers/drv_sensor.h>
#include <lib/drivers/device/Device.hpp>
#include <px4_platform_common/i2c.h>

constexpr px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS] = {
	initI2CBusExternal(4), // GPS1 connector (J13)
	initI2CBusExternal(2), // GPS2 connector (J3) + onboard LIS3MDL
};

bool px4_i2c_device_external(const uint32_t device_id)
{
	// The onboard LIS3MDL (U6, address 0x1E) sits on I2C2, which is also routed
	// to the GPS2 connector. Report just that device as internal so it gets the
	// board rotation (SENS_BOARD_ROT) and internal-mag priority; anything else
	// on I2C2 (e.g. a GPS2 compass) stays external.
	device::Device::DeviceId device_id_mag{};
	device_id_mag.devid_s.bus_type = device::Device::DeviceBusType_I2C;
	device_id_mag.devid_s.bus = 2;
	device_id_mag.devid_s.address = 0x1e;
	device_id_mag.devid_s.devtype = DRV_MAG_DEVTYPE_LIS3MDL;

	if (device_id_mag.devid == device_id) {
		return false;
	}

	device::Device::DeviceId dev_id{};
	dev_id.devid = device_id;
	return px4_i2c_bus_external(dev_id.devid_s.bus);
}
