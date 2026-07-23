/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file stewart_param.c
 *
 * Parameters for Acro mode behavior
 */

/**
 * Deg min per servo of stewart
 *
 *
 * @unit deg
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Stewart platform
 */
PARAM_DEFINE_FLOAT(STEWART_DEG_MIN, 0.f);

/**
 * Deg max per servo of stewart
 *
 *
 * @unit deg
 * @min 0.0
 * @max 1800.0
 * @decimal 1
 * @increment 5
 * @group Stewart platform
 */
PARAM_DEFINE_FLOAT(STEWART_DEG_MAX, 120.f);

/**
 * Relative height of the stewart platform
 *
 * @unit mm
 * @min 40
 * @max 100
 * @decimal 1
 * @increment 1
 * @group Stewart platform
 */
PARAM_DEFINE_FLOAT(STEWART_REL_HGT, 65.0f);

/**
 *
 * Normalized x component of the stewart platform orientation
 * @min -1
 * @max 1
 * @decimal 2
 * @increment 0.01
 * @group Stewart platform
 */
PARAM_DEFINE_FLOAT(STEWART_NORM_NX, 0.0f);

/**
 *
 * Normalized y component of the stewart platform orientation
 * @min -1
 * @max 1
 * @decimal 2
 * @increment 0.01
 * @group Stewart platform
 */
PARAM_DEFINE_FLOAT(STEWART_NORM_NY, 0.0f);

/**
 * Desired roll of the stewart platform
 * @min -45
 * @max  45
 * @decimal 1
 * @increment 1
 * @group Stewart platform
 */
PARAM_DEFINE_FLOAT(STEWART_D_ROLL, 0.0f);

/**
 *
 * Desired pitch of the stewart platform
 * @min -45
 * @max  45
 * @decimal 1
 * @increment 1
 * @group Stewart platform
 */
PARAM_DEFINE_FLOAT(STEWART_D_PITCH, 0.0f);
