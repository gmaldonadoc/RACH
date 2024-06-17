/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2015
 *
 * This file is part of LTE-Sim
 *
 * LTE-Sim is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation;
 *
 * LTE-Sim is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LTE-Sim; if not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <iostream>
#include <string>

/* path */
static std::string path ("/dados/giancarlo/LTE-Sim_RA-Article_M2M_beta/LTE-Sim_RA-Article_M2M/");
/* Version */
static int _VERSION_ = 25;
/*
	adding the complete Discontinuous Reception (DRX) mechanism
  changing the Urban and Suburban Macrocell Propagation Model
  creating the function to set the MCS according to the SINR of the SRS on the uplink
  adding the complete Energy Consumption Model
  adding the Battery Model
  adding D2D Communications
  UM RLC Bugs Correction
  new distribution for Talkspurt/Silence duration to VoIP Traffic application
  new Game Uplink Traffic application
  correct bug in UpdateAverageTransmissionRate on RadioBearer class
  adding downlink direction transmission
  adding bi-directional traffic
  measuring of CQI from the Signal Reference (SR)
  adding new Random-access overload control scheme named CC-RAS
  changing the measurement of effective SINR to uplink (using average)
*/

/* Tracing */
static bool _APP_TRACING_ = true;
static bool _RLC_TRACING_ = true;
static bool _MAC_TRACING_ = false;
static bool _PHY_TRACING_ = false;

/* */
static bool _TEST_BLER_ = false;

/* Channel model type */
static bool _simple_jakes_model_ = false;
static bool _PED_A_ = true;
static bool _PED_B_ = false;
static bool _VEH_A_ = false;
static bool _VEH_B_ = false;

static bool _channel_TU_ = false;
static bool _channel_AWGN_ = true;

/**/
static bool UPLINK = false;
static bool DOWNLINK = false;
static bool RA_ENERGY_TRACE = false;
static bool ONLY_RANDOM_ACCESS = true;
static bool USE_PDCCH_MANAGER = false;
static bool USE_RANDOM_ACCESS = false;
static bool USE_POWER_CONTROL_SRS = false;

static bool USE_EFFICIENT_RA_PREAMBLE = false;

/* Debugging */
//#define APPLICATION_DEBUG
//#define BEARER_DEBUG
//#define RLC_DEBUG
//#define MAC_QUEUE_DEBUG
//#define FLOW_MANAGER_DEBUG
//#define FRAME_MANAGER_DEBUG
//#define ENODEB_DEBUG
//#define UE_DEBUG
//#define PHY_DEBUG
//#define SINR_DEBUG
//#define BLER_DEBUG
//#define MOBILITY_DEBUG
//#define MOBILITY_DEBUG_TAB
//#define HANDOVER_DEBUG
//#define TRANSMISSION_DEBUG
//#define CHANNEL_REALIZATION_DEBUG
//#define TEST_DEVICE_ON_CHANNEL
//#define TEST_START_APPLICATION
//#define TEST_ENQUEUE_PACKETS
//#define TEST_PROPAGATION_LOSS_MODEL
//#define INTERFERENCE_DEBUG
//#define TEST_CQI_FEEDBACKS
//#define SCHEDULER_DEBUG
//#define AMC_MAPPING
//#define PLOT_USER_POSITION
//#define TEST_UL_SINR
//#define TEST_DL_SINR
//#define ULQ_SCHEDULER_DEBUG
//#define PDCCH_DEBUG
//#define DL_RS_DEBUG
//#define UL_SRS_DEBUG
//#define ENERGY_CONSUMPTION_DEBUG
//#define DRX_DEBUG
//#define BATTERY_DEBUG
//#define POWER_CONTROL_DEBUG
//#define RRM_ENTITY_DEBUG
//#define HARQ_HOLDTX_DEBUG
//#define RANDOM_ACCESS_PROCESURE_DEBUG

#endif