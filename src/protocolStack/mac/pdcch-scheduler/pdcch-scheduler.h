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
 * Author: Tiago Pedroso da Cruz de Andrade <tiagoandrade@lrc.ic.unicamp.br>
 */

#ifndef PDCCH_SCHEDULER_H
#define	PDCCH_SCHEDULER_H

#include "../enb-mac-entity.h"
#include "../packet-scheduler/uplink-packet-scheduler.h"
#include "../packet-scheduler/downlink-packet-scheduler.h"

const int HIGH_PRIORITY = 1;
const int LOW_PRIORITY = 0;

class MacEntity;
class UplinkPacketScheduler;
class DownlinkPacketScheduler;

class PdcchScheduler {
public:
  PdcchScheduler();
  virtual ~PdcchScheduler();

  struct DCIToSchedule {
    int m_type; // 0 - UL; 1 - RAR message; 2 - Contention-resolution message; 3 - BSR grant (from SR)
    int m_vectorPosition;
    double m_metric;
    int m_aggregationLevel;
  };

  struct MSG4ToSchedule {
    int m_vectorPosition;
    double m_TDMetric;
    int m_aggregationLevel;
  };

  struct MSG2ToSchedule {
    int m_vectorPosition;
    double m_metric;
    int m_aggregationLevel;
  };

  typedef std::vector<DCIToSchedule*> DCIsToSchedule;
  typedef std::vector<MSG4ToSchedule*> MSGs4ToSchedule;
  typedef std::vector<MSG2ToSchedule*> MSGs2ToSchedule;

  void StartSchedule(void);
  void Schedule(void);
  void DoSchedule(void);
  void StopSchedule();
  void DoStopSchedule();
  
   // Declarar el nuevo método

  void SetMacEntity(MacEntity *mac);
  MacEntity* GetMacEntity(void);

  int GetCurrentCCEsToPDCCH(void);
  int GetMaxUEsToScheduleInUL(void);

  double ComputeULUserTDSchedulingMetric(UplinkPacketScheduler::UserToSchedule *user);
  double ComputeDLFlowTDSchedulingMetric(DownlinkPacketScheduler::FlowToSchedule *flow);
  double ComputeRandomAccessMsg2TDSchedulingMetric(EnbMacEntity::MsgToSchedule *msg2);
  double ComputeRandomAccessMsg4TDSchedulingMetric(EnbMacEntity::MsgToSchedule *msg4);
  double ComputeBSRGrantTDSchedulingMetric(EnbMacEntity::MsgToSchedule *bsrGrant);

  void Msg2NormalPriorityScheduling(void);
  
  void Msg4NormalPriorityScheduling(void);
  void BsrGrantNormalPriorityScheduling(void);

  void Msg2TimePriorityScheduling(void);
  void Msg4TimePriorityScheduling(void);
  
  
  void Msg2PriorityScheduling(void);
  void collision_priority(void);
  void Msg4PriorityScheduling(void);

  void RAP_Scheduling(void); // RA-Prioritized Algorithm (RAP)
  void LTA_Scheduling(void); // Lifetime-Aware Algorithm (LTA)
  void GBRLTA_Scheduling(void); // GBR-Prioritized Lifetime-Aware Algorithm (GBR-LTA)
  void PPA_Scheduling(void);
  void ePPA_Scheduling(void);
  void MinAL_Algorithm(void);
  void collision_detect(void);

  void NMScheduling1(void);

  void SelectMSG2sToSchedule(void);
  void SortMSGs2ToSchedule(MSGs2ToSchedule *MSGs2ToSchedule);

  //void BuildMSGs4Queue(void);

  DCIsToSchedule* GetDCIsToSchedule(void);
  DCIsToSchedule* GetGBRDCIsToSchedule(void);
  DCIsToSchedule* GetNGBRDCIsToSchedule(void);

  MSGs2ToSchedule* GetHighPriorityMSGs2ToSchedule(void);
  MSGs2ToSchedule* GetLowPriorityMSGs2ToSchedule(void);

  MSGs4ToSchedule* GetMSGs4ToSchedule(void);

  void SetLimitUEsToFD(bool value);
  bool GetLimitUEsToFD(void);

  void SetnUEsToDL(int nUEs);

  void SetPDCCHModeType(int type);

  void DeleteDCIsToSchedule(void);
  void ClearDCIsToSchedule(void);

  void DeleteMSGs2ToSchedule(void);
  void ClearMSGs2ToSchedule(void);

  void DeleteMSGs4ToSchedule(void);
  void ClearMSGs4ToSchedule(void);

  void SetMaxMSGs4ToSchedule(int max);

  void SchedulingWithouPDCCH(void);

  bool PDCCHResourceAllocation(int RNTI);

  //void SeparateGBR(UplinkPacketScheduler::UsersToSchedule *usersToSchedule);

  int LinkAdaptation(int RNTI);

  void BestEffortCCEsScheduling(DCIsToSchedule *DCIsToSchedule);
  void NewCCEsScheduling(DCIsToSchedule *DCIsToSchedule);

private:
  MacEntity *m_mac;

  int m_current_pdcch_cces;

  int m_nUlUEsToSchedule;
  int m_nDlUEsToSchedule;
  int m_nMsg2ToSchedule;
  int m_nMsg4ToSchedule;
  int m_nBsrGrantsToSchedule;

  int m_nUlReservedCces;
  int m_nDlReservedCces;
  int m_nMsg2ReservedCces;
  int m_nMsg4ReservedCces;
  int m_nBsrGrantsReservedCces;

  bool m_limitUEsToFD;

  int m_maxGrantsPerRAR;
  int m_maxMSGs4ToSchedule;
  int m_maxBSRGrantsToSchedule;

  int m_modeType; // 1 - Normal; 2 - Time

  DCIsToSchedule *m_DCIsToSchedule;

  DCIsToSchedule *m_GBRDCIsToSchedule;
  DCIsToSchedule *m_NGBRDCIsToSchedule;

  MSGs4ToSchedule *m_MSGs4ToSchedule;

  MSGs2ToSchedule *m_HighPriorityMSGs2ToSchedule;
  MSGs2ToSchedule *m_LowPriorityMSGs2ToSchedule;

  int nUEsToDL;

  int PDCCHResources[84];
  int PDCCHCcesPerUser[200][84];

  int uDl;

  std::vector<int> *UEsToDL;

  void GenerateUserEquipmentDL(int qtd);

  int Calc_Y(int k, int RNTI);
  int Calc_CCE_Index(int m, int i, int RNTI);
  bool PDCCHResourcesAllocation(int RNTI);
  bool PDCCHResourcesAllocation(int m, int i, int RNTI);

  void SelectDCIsToSchedule(void);
  void OrderDCIsByMetric(DCIsToSchedule *DCIsToSchedule);
  void OrderDCIsByAggregationLevel(DCIsToSchedule *DCIsToSchedule);
  void SelectDCIsToSchedule1(void);
};

#endif
