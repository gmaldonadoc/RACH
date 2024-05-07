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

#ifndef RRM_ENTITY_H
#define	RRM_ENTITY_H

#include "../enb-mac-entity.h"
#include "../packet-scheduler/uplink-packet-scheduler.h"
#include "../packet-scheduler/downlink-packet-scheduler.h"
#include "../pdcch-scheduler/pdcch-scheduler.h"
#include "../../../device/ENodeB.h"

class MacEntity;
class UplinkPacketScheduler;
class DownlinkPacketScheduler;
class ENodeB;
class PdcchScheduler;

class RrmEntity {
public:

  RrmEntity();
  virtual ~RrmEntity(void);
  void initialize(ENodeB *);

  void SetUplinkPacketScheduler(UplinkPacketScheduler* s);
  void SetDownlinkPacketScheduler(DownlinkPacketScheduler* s);
  void SetPdcchScheduler();

  PacketScheduler* GetUplinkPacketScheduler(void);

  PacketScheduler* GetDownlinkPacketScheduler(void);

  PdcchScheduler* GetPdcchScheduler();

  void SetULScheduler(void);
  void SetDLScheduler(void);

  void DoSchedule();

private:

  UplinkPacketScheduler *m_uplinkScheduler;
  DownlinkPacketScheduler *m_downlinkScheduler;
  PdcchScheduler *m_pdcchScheduler;

  ENodeB *eNodeB;
};

#endif