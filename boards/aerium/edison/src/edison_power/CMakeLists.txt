############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# edison_power publishes on the libuavcan node owned by uavcannode, so it must
# be compiled with exactly the same libuavcan configuration as
# src/drivers/uavcannode/CMakeLists.txt (STM32 + NuttX driver on this board).
set(LIBDRONECAN_DIR ${PX4_SOURCE_DIR}/src/drivers/uavcan/libdronecan)

px4_add_module(
	MODULE drivers__edison_power
	MAIN edison_power
	COMPILE_FLAGS
		-DUAVCAN_STM32_NUTTX=1
		-DUAVCAN_STM32_NUM_IFACES=${config_uavcan_num_ifaces}
		-DUAVCAN_STM32_TIMER_NUMBER=5
		-DUAVCAN_CPP_VERSION=UAVCAN_CPP03
		-DUAVCAN_DRIVER=uavcan_stm32
		-DUAVCAN_IMPLEMENT_PLACEMENT_NEW=1
		-DUAVCAN_MEM_POOL_BLOCK_SIZE=48
		-DUAVCAN_NO_ASSERTIONS
		-DUAVCAN_PLATFORM=generic
		-Wno-cast-align
		-Wno-deprecated-copy
		-Wno-address-of-packed-member
	INCLUDES
		${PX4_BINARY_DIR}/src/drivers/uavcannode/include/dsdlc_generated
		${LIBDRONECAN_DIR}/libuavcan/include
		${LIBDRONECAN_DIR}/libuavcan/include/dsdlc_generated
		${LIBDRONECAN_DIR}/libuavcan/posix/include
		${PX4_SOURCE_DIR}/src/drivers/uavcan/uavcan_drivers/stm32/driver/include
	SRCS
		EdisonPower.cpp
		EdisonPower.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		drivers__uavcannode
		px4_uavcan_dsdlc
		px4_work_queue
	)
