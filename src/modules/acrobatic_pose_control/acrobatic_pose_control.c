/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file fw_att_control_params.c
 *
 * Parameters defined by the fixed-wing attitude control task
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 */

/*
 * Controller parameters, accessible via MAVLink
 *
 */
/** Fixed-wing quaternion time constant */
PARAM_DEFINE_FLOAT(FW_DQ_Q0_TC, 0.5f);

PARAM_DEFINE_FLOAT(FW_DQ_Q1_TC, 0.5f);

PARAM_DEFINE_FLOAT(FW_DQ_Q2_TC, 0.5f);

PARAM_DEFINE_FLOAT(FW_DQ_Q3_TC, 0.5f);

/** Fixed-wing w velocity control parameters*/
PARAM_DEFINE_FLOAT(FW_DQ_W_I, 0.2f);

PARAM_DEFINE_FLOAT(FW_DQ_W_FF, 0.01f);

PARAM_DEFINE_FLOAT(FW_DQ_W_P, 0.08f);

/** Fixed-wing v velocity control parameters*/
PARAM_DEFINE_FLOAT(FW_DQ_V_I, 0.0f);

PARAM_DEFINE_FLOAT(FW_DQ_V_FF, 0.0f);

PARAM_DEFINE_FLOAT(FW_DQ_V_P, 0.1f);


/** Fixed-wing p velocity control parameters*/
PARAM_DEFINE_FLOAT(FW_DQ_P_I, 0.5f);

PARAM_DEFINE_FLOAT(FW_DQ_P_FF, 0.1f);

PARAM_DEFINE_FLOAT(FW_DQ_P_P, 0.0f);









