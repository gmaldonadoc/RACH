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
 * Author: Tiago Pedroso da Cruz de Andrade
 */

#ifndef UL_LIOUMPAS_A1_PACKET_SCHEDULER_H
#define	UL_LIOUMPAS_A1_PACKET_SCHEDULER_H

#include "uplink-packet-scheduler.h"

class LioumpasA1UplinkPacketScheduler : public UplinkPacketScheduler {
public:
  LioumpasA1UplinkPacketScheduler();
  LioumpasA1UplinkPacketScheduler(UplinkPacketScheduler *sch);
  virtual ~LioumpasA1UplinkPacketScheduler();

  virtual double ComputeSchedulingMetric(RadioBearer *bearer, double spectralEfficiency, int subChannel);
  virtual double ComputeSchedulingMetric(UserToSchedule* user, int subchannel);

  virtual double ComputeTDSchedulingMetric(UserToSchedule *user);
  virtual double ComputeFDSchedulingMetric(UserToSchedule *user, int subchannel);

  virtual void TDScheduling(UsersToSchedule *usersToSchedule);
  virtual void FDScheduling(UsersToSchedule *usersToSchedule, UsersToSchedule *usersHTCToSchedule, UsersToSchedule *usersMTCToSchedule);

  virtual void RBsAllocation();

  virtual int GetAvailablePRBs(void);

private:
  int m_nAvailableRBs;
  UplinkPacketScheduler *objScheduler;
};

#endif