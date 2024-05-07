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

#ifndef ENODEB_H_
#define ENODEB_H_

#include "NetworkNode.h"
#include "../channel/LteChannel.h"

class UserEquipment;
class Gateway;

class PacketScheduler;
class PdcchScheduler;
class RrmEntity;

class InfiniteBuffer;
class VoIP;
class CBR;
class TraceBased;
class WEB;
class UplinkPacketScheduler;
class DownlinkPacketScheduler;
class QoSParameters;

class ENodeB : public NetworkNode {
public:

  struct UserEquipmentRecord {
    UserEquipmentRecord();
    virtual ~UserEquipmentRecord();
    UserEquipmentRecord(UserEquipment *UE);

    UserEquipment *m_UE;
    void SetUE(UserEquipment *UE);
    UserEquipment* GetUE(void) const;

    std::vector<int> m_cqiFeedback;
    void SetCQI(std::vector<int> cqi);
    std::vector<int> GetCQI(void) const;

    int m_schedulingRequest; // in bytes
    void SetSchedulingRequest(int r);
    int GetSchedulingRequest(void);

    int m_averageSchedulingGrants; // in bytes
    void UpdateSchedulingGrants(int b);
    int GetSchedulingGrants(void);

    int m_ulMcs;
    void SetUlMcs(int mcs);
    int GetUlMcs(void);

    std::vector<double> m_uplinkChannelStatusIndicator;
    void SetUplinkChannelStatusIndicator(std::vector<double> vet);
    std::vector<double> GetUplinkChannelStatusIndicator(void) const;

    int m_scheduledData; //scheduled Bytes in a TTI (modified by the schedulers each TTI)

    double m_averageUERate; // moving average rate record for the TD Scheduler in bps
    double GetUERate(void);

    double m_averageUERateTD; // moving average rate for the FD Scheduler in bps
    double GetUERateTD(void);

    void UpdateAverageSchRate(void); //This is currently used for updating both TD and FD moving average rates

    void SetInitialUERate(double r);

    double m_lastUpdate; // time in seconds!
    void SetLastUpdate(); //Set the current time in seconds
    double GetLastUpdate(void); //return the last update time in seconds

    double m_GBR;
    void SetGBR(double GBR);
    double GetGBR(void) const;

    double m_maximumDelay;
    void SetMaximumDelay(double maximumDelay);
    double GetMaximumDelay(void);

    int m_semiRBs;
    int m_semiMCS;
    int m_semiSchTime;
    int m_semiSchPeriod;
    int m_semiReliase;
    int m_semiWait;
    void SetSemiSchConfig(int startTime, int schPeriod);
    double GetSemiSchTime(void);
    void UpdateSemiSchTime(void);

    double m_CqiFromReferenceSignal;

    double m_currentPowerHeadroom;
    void SetPowerHeadroom(double phr);
    double GetPowerHeadroom(void);

    UserEquipment *m_d2dCommunicationUE;
    void SetD2DCommunicationUE(UserEquipment *UE);
    UserEquipment* GetD2DCommunicationUE(void);

    /*HARQ Process*/
    PacketBurst *m_harqBuffer[8];
  };

  typedef std::vector<UserEquipmentRecord*> UserEquipmentRecords;

  enum DLSchedulerType {
    DLScheduler_TYPE_NONE,
    DLScheduler_TYPE_MAXIMUM_THROUGHPUT,
    DLScheduler_TYPE_PROPORTIONAL_FAIR,
    DLScheduler_TYPE_FLS,
    DLScheduler_TYPE_MLWDF,
    DLScheduler_TYPE_EXP,
    DLScheduler_LOG_RULE,
    DLScheduler_EXP_RULE
  };

  enum ULSchedulerType {
    ULScheduler_TYPE_NONE,
    ULScheduler_TYPE_MAXIMUM_THROUGHPUT,
    ULScheduler_TYPE_FME,
    ULScheduler_TYPE_ROUNDROBIN,
    ULScheduler_TYPE_ZBQoS,
    ULScheduler_TYPE_LIOUMPAS_A1,
    ULScheduler_TYPE_LIOUMPAS_A2
  };

  enum ULTDSchedulerType {
    ULTDScheduler_TYPE_NONE,
    ULTDScheduler_TYPE_MT,
    ULTDScheduler_TYPE_PF,
    ULTDScheduler_TYPE_RR,
    ULTDScheduler_TYPE_ZBQoS
  };

  enum ULFDSchedulerType {
    ULFDScheduler_TYPE_NONE,
    ULFDScheduler_TYPE_FME_MT,
    ULFDScheduler_TYPE_FME_PF,
    ULFDScheduler_TYPE_FME_RR
  };

  ENodeB();
  ENodeB(int idElement, Cell *cell);
  ENodeB(int idElement, Cell *cell, double posx, double posy);

  virtual ~ENodeB();

  void RegisterUserEquipment(UserEquipment *UE);
  void DeleteUserEquipment(UserEquipment *UE);
  int GetNbOfUserEquipmentRecords(void);
  void CreateUserEquipmentRecords(void);
  void DeleteUserEquipmentRecords(void);
  UserEquipmentRecords* GetUserEquipmentRecords(void);
  UserEquipmentRecord* GetUserEquipmentRecord(int idUE);

  void SetDLScheduler(DLSchedulerType type);
  PacketScheduler* GetDLScheduler(void) const;
  void SetULScheduler(ULSchedulerType type);
  void SetULScheduler(void);
  PacketScheduler* GetULScheduler(void) const;

  void SetPdcchScheduler(void);
  PdcchScheduler* GetPdcchScheduler(void) const;

  void ResourceBlocksAllocation();
  void UplinkResourceBlockAllocation();
  void DownlinkResourceBlokAllocation();

  //Debug
  void Print(void);

  void SetQoSParametersforUERecord(int idUE, QoSParameters* QoSP);

  void SetSemiSchConfigUERecord(int idUE, int startTime, int schPeriod);
  void SemiSchRBCalculation(UserEquipmentRecord *record);

  void SchedulingRequestConfiguration(int nbSRsPerPUCCHRegion);

  void SetSinrFromReferenceSignal(int idUE, double CQI);

  void SetP0NominalPusch(double power);
  void SetP0NominalPucch(double power);
  double GetP0NominalPusch(void);
  double GetP0NominalPucch(void);

  void SetReferenceSensitivityPowerLevel(double power);
  double GetReferenceSensitivityPowerLevel(void);

  void SetAlphaPL(double alpha);
  double GetAlphaPL(void);

  void SetReferenceSignalPower(double power);
  double GetReferenceSignalPower(void);

  void SetPreambleInitialReceivedTargetPower(double power);
  double GetPreambleInitialReceivedTargetPower(void);

  void SetPowerRampingStep(double ramping);
  double GetPowerRampingStep(void);

  void PowerHeadroomReport(int idUE, double phr);

  //[DRX]
  void InitDRXCycle(void);

  void InitConsumptionEnergyCycle(void);

  void SetRrmEntity(void);
  RrmEntity *GetRrmEntity(void);

  //typedef std::pair<NetworkNode*, NetworkNode*> ChannelEstimationId;
  //typedef std::map <ENodeB::ChannelEstimationId, std::vector<double>*> ChannelEstimationMap;

private:
  UserEquipmentRecords *m_userEquipmentRecords;

  int SRperiodicity; // in ms

  double m_refSen; // SetReferenceSignalPower in dBm

  /* SIB2 */
  double P0_Nominal_PUSCH; // base level in dBm/PRB (-126dBm to +24dBm)
  double P0_Nominal_PUCCH;
  double alpha_PL; // in linear (0, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1)
  double referenceSignalPower; // in dBm/signal

  double preambleInitialReceivedTargetPower; //in dBm (-120, -118, -116, -114, -112, -110, -108, -106, -104, -102, -100, -98, -96, -94, -92, -90)
  double powerRampingStep; // in dB (0, 2, 4, 6)

  RrmEntity *m_RrmEntity;

  ENodeB::DLSchedulerType DLtype;
  ENodeB::ULSchedulerType ULtype;
};

#endif /* ENODEB_H_ */
