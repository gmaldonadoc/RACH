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

#ifndef UE_MAC_ENTITY_H
#define UE_MAC_ENTITY_H

#include <list>
#include <vector>
#include "mac-entity.h"
#include "../../core/eventScheduler/event.h"

class CqiIdealControlMessage;
class PdcchMapIdealControlMessage;
class BufferStatusReportingIdealControlMessage;
class RAPreambleIdealControlMessage;
class RAResponseIdealControlMessage;
class ConnectionRequestIdealControlMessage;
class ContentionResolutionIdealControlMessage;
class HarqIdealControlMessage;
class SchedulingRequestPUCCHIdealControlMessage;
class BsrGrantIdealControlMessage;

class ENodeB;

class PacketBurst;

/*
 * This class implements the MAC layer of the UE device
 */
class UeMacEntity : public MacEntity {
public:
  UeMacEntity(void);
  virtual ~UeMacEntity(void);

  void SendSchedulingRequest(void);

  void SendBufferStatusReport(int how);

  void ReceiveUplinkAllocationMap(std::vector<int> prbs, int mcs, int id);

  void ScheduleUplinkTransmission(std::vector<int> prbs, int mcs, int id);

  void ScheduleUplinkTransmissionSemiSch();

  double GetPeriodicBSRInterval(void);
  void SetPeriodicBSRInterval(double time);

  void CheckForDropPackets(void);

  //
  void StartRAProcedure(void);
  void StopRAProcedure(bool failure);
  bool GetDoingRAProcedure(void);
  void SendRAPreamble(void);

  void ReceiveRandomAccessResponseIdealControlMessage(RAResponseIdealControlMessage *msg);
  void ReadyRandomAccessResponseIdealControlMessage(RAResponseIdealControlMessage *msg);

  void TimeoutRAResponseWindowTimer(void);
  void StartRAResponseWindowTimer(void);

  void TimeoutContentionResolutionTimer(int counter);
  void StartContentionResolutionTimer(void);

  void AssemblyConnectionRequestIdealControlMessage(void);

  void AssemblySchedulingRequestIdealControlMessage(void);

  void ReceiveContentionResolutionIdealControlMessage(ContentionResolutionIdealControlMessage *contentionResolution);
  void ReadyContentionResolutionIdealControlMessage(ContentionResolutionIdealControlMessage *contentionResolution);

  void ReceiveHarqIdealControlMessage(HarqIdealControlMessage *harq);
  void ReadyHarqIdealControlMessage(HarqIdealControlMessage *harq);

  bool GetInBackoff(void);
  void StartBackoffTimer();
  void TimeoutBackoffTimer();
  void SetNbAvailablePreambles(int nbAvailablePreambles);
  void SetRARWindowSize(int RARWindowSize);

  void SetEABFactor(double factor);
  void SetEABTime(double time);

  bool getRAStopped() {
    return m_RAStopped;
  }

  void TriggerBufferStatusReport(int BSRType);
  void StartRetxBSRTimer(void);
  void TimeoutRetxBSRTimer(void);

  void CheckForPagingMessage(void);
  void StartCheckForPagingMessage(void);
  void CheckForSystemInformation(void);

  virtual int GetRealNbOfRBsForULScheduling(void);

  virtual int getCurrentPDCCHCCEs(void);
  virtual void updateCurrentPDCCHCCEs(int cces);

  void TimeoutSchedulingRequestTimer(void);

  void TimeoutSchedulingRequestProhibitTimer(void);

  void ReceiveBsrGrantIdealControlMessage(BsrGrantIdealControlMessage *bsrGrantMessage);
  void ReadyBsrGrantIdealControlMessage(void);

  void TimeoutPeriodicBSRTimer(void);

  int GetPreambleTransmissionCounter(void);

  void SetPowerHeadroom(double phr);

  /* To DRX */
  bool executingRA();
  void SetExecutingRA(bool e);

  void SetPreambleSequence(int preamble);

  void updateCounterMatchedRA_RNTI(RAResponseIdealControlMessage *msg);
  void setCounterMatchedRA_RNTI(int x);
  void resetCounterMatchedRA_RNTI();
  int getCounterMatchedRA_RNTI();
  double getExpectedPreambleCollisions();
  void setExpectedPreambleCollisions(double);

  double calculateExpectedCollisions(int);

  int m_temp_enb_users_trying_RA;
  int m_temp_enb_preambles_collided;
  int m_temp_enb_preambles_distinct;

private:
  double m_BSRInterval; //seconds
  double m_expentAtRAStart;
  double m_instantAtStart;

  bool m_hasPreambleWaiting;

  bool m_BackoffTimer;

  int m_preambleTransmissionCounter;
  int m_backoffIndicator;

  int m_raRNTI;
  int m_raRNTI_Using;
  int m_preamble;

  int m_counterMatchedRA_RNTI;
  double m_instantPreambleSent;
  double m_expectedPreambleCollisions;

  bool m_inCollisionAvoidance;
  int m_backoffExponent;

  bool m_RAStopped;

  int m_msg3TransmissionCounter;

  int m_waitToSendPreamble;

  // NEW
  bool m_noSensitiveBlocked;

  Event *m_ContentionResolutionTimerHandler;
  bool m_ContentionResolutionTimer;

  Event *m_RAResponseWindowTimerHandler;
  bool m_RAResponseWindowTimer;

  Event *m_retxBSRTimerHandler;
  bool m_retxBSRTimer;

  double m_retxBSR_Time;
  bool m_triggered_BSR;

  bool m_information_changed;
  bool m_eab_barred;

  bool m_hasSchedulingRequestWaiting;
  bool m_isSchedulingRequestProhibitPeriod;

  double m_currentPowerHeadroom;

  bool m_executingRA;

  bool m_fixedPreamble;

  bool m_premature_harq_stop;

  /*HARQ Process*/
  PacketBurst *m_harqBuffer[8];
};

#endif /* UE_MAC_ENTITY_H */
