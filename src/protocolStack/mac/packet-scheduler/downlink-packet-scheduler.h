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


#ifndef DOWNLINKPACKETSCHEDULER_H_
#define DOWNLINKPACKETSCHEDULER_H_

#include "packet-scheduler.h"
#include "../../../device/ENodeB.h"

class DownlinkPacketScheduler : public PacketScheduler {
public:
  DownlinkPacketScheduler();
  ~DownlinkPacketScheduler();

  void SelectFlowsToSchedule();

  void DoSchedule(void);
  void DoStopSchedule(void);

  void SetDownlinkTDScheduler(ENodeB::DLSchedulerType);
  void SetDownlinkFDScheduler(ENodeB::DLSchedulerType);

  DownlinkPacketScheduler* GetDownlinkFDScheduler(void);
  DownlinkPacketScheduler* GetDownlinkTDScheduler(void);

  virtual int GetAvailablePRBs(void);
  FlowsToSchedule* GetFlowsMTCToSchedule(void);
  void ClearFlowsMTCToSchedule(void);
  int GetFlowsFD() const;

  void SetnFlowsFD(int);

  void ClearFlowsHTCToSchedule();
  FlowsToSchedule* GetFlowsHTCToSchedule(void);

  virtual void FDScheduling(FlowsToSchedule *, FlowsToSchedule *, FlowsToSchedule *);
  virtual void TDScheduling(FlowsToSchedule *);

  void limitUEsInFD();

  virtual void RBsAllocation();
  virtual double ComputeSchedulingMetric(RadioBearer *, double, int);

  void UpdateAverageTransmissionRate(void);
  void UpdateAverageScheduladedRate(void);

  void DoSchedulingTD();
  void DoSchedulingFD();

private:

  int m_nUEsFD;
  int m_nAvailableRBs;
  int m_nSemiSchRBs;
  int m_nUEsSemiSch;

  ENodeB::DLSchedulerType m_dlTDSchedulerType;
  ENodeB::DLSchedulerType m_dlFDSchedulerType;

  DownlinkPacketScheduler *m_downlinkTDScheduler;
  DownlinkPacketScheduler *m_downlinkFDScheduler;

  FlowsToSchedule *m_flowsHTCToSchedule;
  FlowsToSchedule *m_flowsMTCToSchedule;
  FlowsToSchedule *m_flowsToSchedule;
};

#endif /* DOWNLINKPACKETSCHEDULER_H_ */
