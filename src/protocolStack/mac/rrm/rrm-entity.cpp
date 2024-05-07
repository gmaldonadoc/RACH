
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
 * Authors: Silvana Trindade <silvana@lrc.ic.unicamp.br>,
 *          Tiago Pedroso da Cruz de Andrade <tiagoandrade@lrc.ic.unicamp.br>, and
 *          Carlos A. Astudillo Trujillo <castudillo@ieee.org>
 */

#include "rrm-entity.h"

#include "../packet-scheduler/enhanced-uplink-packet-scheduler.h"
#include "../packet-scheduler/roundrobin-uplink-packet-scheduler.h"
#include "../packet-scheduler/mt-uplink-packet-scheduler.h"
#include "../packet-scheduler/dl-pf-packet-scheduler.h"
#include "../packet-scheduler/dl-mlwdf-packet-scheduler.h"
#include "../packet-scheduler/dl-exp-packet-scheduler.h"
#include "../packet-scheduler/dl-fls-packet-scheduler.h"
#include "../packet-scheduler/exp-rule-downlink-packet-scheduler.h"
#include "../packet-scheduler/log-rule-downlink-packet-scheduler.h"

RrmEntity::RrmEntity() {
}

RrmEntity::~RrmEntity(void) {
}

void RrmEntity::initialize(ENodeB *e) {
  eNodeB = e;
}

void RrmEntity::SetUplinkPacketScheduler(UplinkPacketScheduler* s) {
  m_uplinkScheduler = s;
}

void RrmEntity::SetDownlinkPacketScheduler(DownlinkPacketScheduler* s) {
  m_downlinkScheduler = s;
}

void RrmEntity::SetPdcchScheduler() {
  EnbMacEntity *mac = (EnbMacEntity*) eNodeB->GetProtocolStack()->GetMacEntity();

  m_pdcchScheduler = new PdcchScheduler();
  m_pdcchScheduler->SetMacEntity(mac);
  mac->SetPdcchScheduler(m_pdcchScheduler);
}

PacketScheduler* RrmEntity::GetUplinkPacketScheduler(void) {
  EnbMacEntity *mac = (EnbMacEntity*) eNodeB->GetProtocolStack()->GetMacEntity();
  return mac->GetUplinkPacketScheduler();
}

PacketScheduler* RrmEntity::GetDownlinkPacketScheduler(void) {
  EnbMacEntity *mac = (EnbMacEntity*) eNodeB->GetProtocolStack()->GetMacEntity();
  return mac->GetDownlinkPacketScheduler();
}

PdcchScheduler* RrmEntity::GetPdcchScheduler() {
  EnbMacEntity *mac = (EnbMacEntity*) eNodeB->GetProtocolStack()->GetMacEntity();
  return mac->GetPdcchScheduler();
}

void RrmEntity::SetULScheduler() {
  EnbMacEntity *mac = (EnbMacEntity*) eNodeB->GetProtocolStack()->GetMacEntity();

  m_uplinkScheduler = new UplinkPacketScheduler();
  m_uplinkScheduler->SetMacEntity(mac);
  mac->SetUplinkPacketScheduler(m_uplinkScheduler);
}

void RrmEntity::SetDLScheduler() {
  EnbMacEntity *mac = (EnbMacEntity*) eNodeB->GetProtocolStack()->GetMacEntity();

  m_downlinkScheduler = new DownlinkPacketScheduler();
  m_downlinkScheduler->SetMacEntity(mac);
  mac->SetDownlinkPacketScheduler(m_downlinkScheduler);
}

void RrmEntity::DoSchedule() {
  //if (GetUplinkPacketScheduler() != NULL && GetDownlinkPacketScheduler() != NULL && eNodeB->GetNbOfUserEquipmentRecords() > 0) {
    if (DOWNLINK == true) {
      m_downlinkScheduler->SelectFlowsToSchedule();
    }
    if (UPLINK == true) {
      m_uplinkScheduler->SelectUsersToSchedule();
    }

    if (DOWNLINK == true) {
      printf("flowsToSchedule*= %d\n", m_downlinkScheduler->GetFlowsToSchedule()->size());
    }

    if (UPLINK == true) {
      printf("UsersToSchedule*= %d\n", m_uplinkScheduler->GetUsersToSchedule()->size());
    }
  //}
  
  GetPdcchScheduler()->StartSchedule();
  
  //if (GetUplinkPacketScheduler() != NULL && GetDownlinkPacketScheduler() != NULL && eNodeB->GetNbOfUserEquipmentRecords() > 0) {
    if (DOWNLINK == true) {
      m_downlinkScheduler->DoSchedulingTD();
    }
    if (UPLINK == true) {
      m_uplinkScheduler->DoSchedulingTD();
    }
  //}
  
  GetPdcchScheduler()->Schedule();

  //if (GetUplinkPacketScheduler() != NULL && GetDownlinkPacketScheduler() != NULL && eNodeB->GetNbOfUserEquipmentRecords() > 0) {
    if (DOWNLINK == true) {
      m_downlinkScheduler->DoSchedulingFD();
    }
    if (UPLINK == true) {
      m_uplinkScheduler->DoSchedulingFD();
    }

    if (DOWNLINK == true) {
      m_downlinkScheduler->DoStopSchedule();
    }
    if (UPLINK == true) {
      m_uplinkScheduler->DoStopSchedule();
    }
  //}
  
  GetPdcchScheduler()->StopSchedule();
  
  //if (GetUplinkPacketScheduler() != NULL && GetDownlinkPacketScheduler() != NULL && eNodeB->GetNbOfUserEquipmentRecords() > 0) {
    if (DOWNLINK == true) {
      m_downlinkScheduler->DeleteFlowsToSchedule();
    }
    if (UPLINK == true) {
      m_uplinkScheduler->DeleteUsersToSchedule();
    }
  //}
}