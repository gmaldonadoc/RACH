/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2012
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
 * Author: Carlos A. Astudillo Trujillo <castudillo@ieee.org>
 * Laboratório de Redes de Computadores (LRC)
 * Universidade Estadual de Campinas
 * Campinas, Brazil
 */
#ifndef ZBQoS_UPLINK_PACKET_SCHEDULER_H_
#define ZBQoS_UPLINK_PACKET_SCHEDULER_H_

#include <vector>
#include "uplink-packet-scheduler.h"

class ZBQoSUplinkPacketScheduler : public UplinkPacketScheduler {
public:
  ZBQoSUplinkPacketScheduler();
  ZBQoSUplinkPacketScheduler(UplinkPacketScheduler *sch);
  virtual ~ZBQoSUplinkPacketScheduler();

  virtual double ComputeSchedulingMetric(RadioBearer *bearer, double spectralEfficiency, int subChannel);
  virtual double ComputeSchedulingMetric(UserToSchedule *user, int subchannel);

  virtual double ComputeTDSchedulingMetric(UserToSchedule *user);
  virtual double ComputeFDSchedulingMetric(UserToSchedule *user, int subchannel);

  virtual void TDScheduling(UsersToSchedule *usersToSchedule);
  virtual void FDScheduling(UsersToSchedule *usersToSchedule, UsersToSchedule *usersHTCToSchedule, UsersToSchedule *usersMTCToSchedule);

  virtual void RBsAllocation(void);

  virtual int GetAvailablePRBs(void);

private:
  int m_nAvailableRBs;
  UplinkPacketScheduler *objScheduler;
};

#endif