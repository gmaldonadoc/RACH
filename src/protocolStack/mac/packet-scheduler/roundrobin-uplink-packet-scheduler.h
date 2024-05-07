/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010,2011,2012,2013 TELEMATICS LAB, Politecnico di Bari
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
 * Author: Giuseppe Piro <g.piro@poliba.it>
 */


#ifndef ROUNDROBIN_UPLINK_PACKET_SCHEDULER_H_
#define ROUNDROBIN_UPLINK_PACKET_SCHEDULER_H_

#include "uplink-packet-scheduler.h"

class RoundRobinUplinkPacketScheduler : public UplinkPacketScheduler {
public:
  RoundRobinUplinkPacketScheduler();
  RoundRobinUplinkPacketScheduler(UplinkPacketScheduler *sch);
  virtual ~RoundRobinUplinkPacketScheduler();

  void RBsAllocation();

  virtual double ComputeSchedulingMetric(RadioBearer *bearer, double spectralEfficiency, int subChannel);
  virtual double ComputeSchedulingMetric(UserToSchedule* user, int subchannel);

  //New Functions (Carlos A.)
  virtual double ComputeTDSchedulingMetric (UserToSchedule* user); //Receive user pointer (Carlos A.).
  virtual double ComputeFDSchedulingMetric (UserToSchedule* user, int subchannel); //Receive user pointer and PRB index (Carlos A.).
  virtual void TDScheduling(UsersToSchedule *usersToSchedule); //Function for executing TD Scheduling (Carlos A.)
  virtual void FDScheduling(UsersToSchedule *usersToSchedule, UsersToSchedule *usersHTCToSchedule, UsersToSchedule *usersMTCToSchedule); //function for executing FD scheduling (Carlos A.)
  //********************

  virtual int GetAvailablePRBs(void);
  
private:
  int m_roundRobinId;
  int m_nAvailableRBs;
  UplinkPacketScheduler *objScheduler;
};

#endif /* ROUNDROBIN_UPLINK_PACKET_SCHEDULER_H_ */
