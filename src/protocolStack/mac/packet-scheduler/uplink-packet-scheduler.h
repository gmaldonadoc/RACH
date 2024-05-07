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

#ifndef UPLINKPACKETSCHEDULER_H_
#define UPLINKPACKETSCHEDULER_H_

#include "packet-scheduler.h"
#include "../../../device/ENodeB.h"

class UplinkPacketScheduler : public PacketScheduler {
public:
  UplinkPacketScheduler();
  virtual ~UplinkPacketScheduler();

  struct UserToSchedule {
    NetworkNode* m_userToSchedule;
    int m_transmittedData; //bytes
    int m_dataToTransmit; //bytes
    double m_averageSchedulingGrant; // in bytes

    int m_selectedMCS;
    std::vector<int> m_listOfAllocatedRBs;
    std::vector<double> m_channelCondition;

    double m_averageUERate; //Averague UE rate bps ! TODO: I think that it is in Kbps
    double m_gbr; // TODO:Implement this (Carlos A.)
    double real_HoL; //in ms (Carlos A.)
    double m_maximumDelay; //TODO: Implement this, in ms (Carlos A.)
    double m_TDMetric; //
    int m_bearerQCI;

    bool m_allowToSchedule;

    double m_currentPowerHeadroom;
  };

  typedef std::vector<UserToSchedule*> UsersToSchedule;

  void CreateUsersToSchedule(void);
  void DeleteUsersToSchedule(void);
  void ClearUsersToSchedule();
  UsersToSchedule* GetUsersToSchedule(void);

  void ClearUsersHTCToSchedule();
  UsersToSchedule* GetUsersHTCToSchedule(void);

  void ClearUsersMTCToSchedule();
  UsersToSchedule* GetUsersMTCToSchedule(void);

  void SelectUsersToSchedule(void);

  //New Functions
  virtual double ComputeTDSchedulingMetric(UserToSchedule *user);
  virtual double ComputeFDSchedulingMetric(UserToSchedule *user, int subchannel);
  virtual void TDScheduling(UsersToSchedule *usersToSchedule);
  virtual void FDScheduling(UsersToSchedule *usersToSchedule, UsersToSchedule *usersHTCToSchedule, UsersToSchedule *usersMTCToSchedule);
  //********************

  virtual void DoSchedule(void);
  virtual void DoStopSchedule(void);

  virtual void RBsAllocation();
  virtual double ComputeSchedulingMetric(RadioBearer *bearer, double spectralEfficiency, int subChannel);
  virtual double ComputeSchedulingMetric(UserToSchedule* user, int subchannel);

  void SetnUsersFD(int usersFD);
  int GetUsersFD() const;

  void UpdateAverageScheduladedRate(void);
  double ZShapeFunction(double max, double x, double a, double b);

  void limitUEsInFD(void);
  void limitUEs1(int maxUEs);

  void SetUplinkTDScheduler(ENodeB::ULSchedulerType type);
  UplinkPacketScheduler* GetUplinkTDScheduler(void);

  void SetUplinkFDScheduler(ENodeB::ULSchedulerType type);
  UplinkPacketScheduler* GetUplinkFDScheduler(void);

  int GetSemiSchRBs();

  virtual int GetAvailablePRBs(void);

  void DoSchedulingTD(void);
  void DoSchedulingFD(void);
  
  bool* GetPRBsAllocatedToRetransmission(void) {
    return m_PRBsAllocatedToRetransmission;
  }

private:
  int m_nUEsFD;

  int m_nAvailableRBs;

  int m_nSemiSchRBs;

  int m_nUEsSemiSch;

  ENodeB::ULSchedulerType m_ulTDSchedulerType;
  ENodeB::ULSchedulerType m_ulFDSchedulerType;

  UplinkPacketScheduler *m_uplinkTDScheduler;
  UplinkPacketScheduler *m_uplinkFDScheduler;

  UsersToSchedule *m_usersToSchedule;
  UsersToSchedule *m_usersHTCToSchedule;
  UsersToSchedule *m_usersMTCToSchedule;
  
  UsersToSchedule *m_usersToRetransmissionSchedule;
  
  bool m_PRBsAllocatedToRetransmission[110];
};

#endif