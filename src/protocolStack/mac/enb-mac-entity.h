#ifndef ENB_MAC_ENTITY_H
#define ENB_MAC_ENTITY_H

#include <list>
#include <vector>
#include <deque>
#include <unistd.h>
#include "mac-entity.h"

#include "../../core/eventScheduler/simulator.h"

/*
 * This class implements the MAC layer of the eNodeB device
 */

class PacketScheduler;
class PdcchScheduler;
class IdealControlMessage;
class CqiIdealControlMessage;
class PdcchMapIdealControlMessage;
class BufferStatusReportingIdealControlMessage;
class RAPreambleIdealControlMessage;
class RAResponseIdealControlMessage;
class ConnectionRequestIdealControlMessage;
class ContentionResolutionIdealControlMessage;
class HarqIdealControlMessage;
class SchedulingRequestIdealControlMessage;
class SchedulingRequestPUCCHIdealControlMessage;
class BsrGrantIdealControlMessage;

class EnbMacEntity : public MacEntity
{
public:
  EnbMacEntity(void);
  virtual ~EnbMacEntity(void);

  void SetUplinkPacketScheduler(PacketScheduler *s);
  void SetDownlinkPacketScheduler(PacketScheduler *s);
  void SetPdcchScheduler(PdcchScheduler *s);

  PacketScheduler *GetUplinkPacketScheduler(void);

  PacketScheduler *GetDownlinkPacketScheduler(void);
  PdcchScheduler *GetPdcchScheduler();

  void ReceiveCqiIdealControlMessage(CqiIdealControlMessage *msg);
  void SendPdcchMapIdealControlMessage(PdcchMapIdealControlMessage *pdcchMsg);

  //
  void ReceiveBufferStatusReportingIdealControlMessage(BufferStatusReportingIdealControlMessage *msg);
  void ReceiveBufferStatusReportingMessage(int source, int bufferSize);

  //
  void ReceiveRandomAccessPreambleIdealControlMessage(RAPreambleIdealControlMessage *RAPreamble);
  void ReadyRandomAccessPreambleIdealControlMessage();
  void ReadyRandomAccessPreambleIdealControlMessage1(RAPreambleIdealControlMessage *RAPreamble);

  //
  void ReceiveConnectionRequestIdealControlMessage(ConnectionRequestIdealControlMessage *connectionRequest);
  void ReadyConnectionRequestIdealControlMessage(ConnectionRequestIdealControlMessage *connectionRequest);

  //
  void ReceiveSchedulingRequestIdealControlMessage(SchedulingRequestIdealControlMessage *connectionRequest);
  void ReadySchedulingRequestIdealControlMessage();
  void ReadySchedulingRequestIdealControlMessage1(SchedulingRequestIdealControlMessage *schedulingRequest);
  void FillCounterUEsFailedPreambles(RAPreambleIdealControlMessage *RAPreamble);

  void AssemblyContentionResolutionIdealControlMessage(NetworkNode *destination);

  void AssemblyBsrGrantIdealControlMessage(NetworkNode *destination);

  void AssemblyRandomAccessResponseIdealControlMessage(int RARNTI, int preamble, NetworkNode *destination);

  void SendRandomAccessResponseIdealControlMessage(RAResponseIdealControlMessage *RAResponse);

  void AllocationResourceToMsg3Retransmission(void);
  void AllocationResourceToDataTransmission(void);
  void AllocationResourceToDataTransmission(int type);

  void SetRARWindowSize(int RARWindowSize);
  void SetBackoffIndicator(int index);
  
  // Definir las prioridades
  static const int LOW_PRIORITY = 0;
  static const int HIGH_PRIORITY = 1;
  // *******************Nuevo método para recibir preámbulos de acceso aleatorio********************//
  //void EnbMacEntity::ReceiveRandomAccessPreambleMessage(RAPreambleIdealControlMessage *RAPreamble);
  void ReceiveRandomAccessPreambleMessage(RAPreambleIdealControlMessage *RAPreamble);
  
  // *******************Nuevo método para procesar preámbulos de acceso aleatorio********************//
  //void EnbMacEntity::ProcessRandomAccessPreambleMessages(int raSlot);
  void ProcessRandomAccessPreambleMessages(int raSlot);
  
  // *******************Nuevo método para preparar el mensaje de RA********************//
  //void EnbMacEntity::PrepareRandomAccessResponseMessage(std::vector<RAPreambleIdealControlMessage *> *processedRAPreambleContainer);
  void PrepareRandomAccessResponseMessage(int raSlot, std::vector<RAPreambleIdealControlMessage *> *processedRAPreambleContainer);

  virtual int getCurrentPDCCHCCEs(void);
  virtual void updateCurrentPDCCHCCEs(int cces);

  virtual int GetRealNbOfRBsForULScheduling(void);

  void ResetAllocatedULRBs(void);

  int getAllocatedResourcesToMsg3ReTX();

  struct MsgToSchedule
  {
    int m_RARNTI;
    int m_preamble;
    double m_timeArrived;
    bool m_scheduled;
    double m_timePriority;
    double m_delay;
    double m_ar;
    bool m_collision;
    int m_priority; // 1 para alta prioridad, 0 para baja prioridad
    
    NetworkNode *m_destination;
  };

  typedef std::deque<MsgToSchedule *> MSGsToSchedule;

  MSGsToSchedule *getMSGs2ToSchedule(void);
  MSGsToSchedule *getMSGs4ToSchedule(void);

  MSGsToSchedule *GetBsrGrantsToSchedule(void);

  void CleanRandomAccessMessagensContainer(void);
  void CleanRandomAccessMessagesQueue(void);

  void CheckForDelayedMSGs2OfRandomAccess(void);
  void CheckForDelayedMSGs4OfRandomAccess(void);

  bool GetInformationChanged(void);

  void ReceiveSchedulingRequestPUCCHIdealControlMessage(SchedulingRequestPUCCHIdealControlMessage *schedulingRequestMessage);
  void ReadySchedulingRequestPUCCHIdealControlMessage(SchedulingRequestPUCCHIdealControlMessage *schedulingRequestMessage);

  void ProcessPreamblesReceived(void);

  std::vector<RAPreambleIdealControlMessage *> *GetM_RAPreambleContainer();

  std::vector<int> GetAllPreambles();

  int *Getm_preamblesReceiveM_PreamblesReceived()
  {
    return m_preamblesReceived;
  }

private:
  void TimeoutMonitoringCycle(void);
  void EABChangeClassBarred(void);

  std::vector<int> all_preambles;

  MSGsToSchedule *m_msgs2ToSchedule;
  MSGsToSchedule *m_msgs4ToSchedule;

  MSGsToSchedule *m_bsrGrantsToSchedule;

  int m_preamblesReceived[64];

  PacketScheduler *m_uplinkScheduler;
  PacketScheduler *m_downlinkScheduler;

  PdcchScheduler *m_pdcchScheduler;

  std::vector<RAPreambleIdealControlMessage *> *m_RAPreambleContainer;

  std::vector<ConnectionRequestIdealControlMessage *> *m_ConnectionRequestContainer;

  std::vector<SchedulingRequestIdealControlMessage *> *m_SchedulingRequestContainer;

  int m_unsuccessful_ra_counter;
  int m_total_ra_counter;

  int m_current_pdcch_cces;
  int m_allocated_ul_rbs;

  int m_allocated_ul_prbs_msg3_retransmition;

  int m_preamble_transmissions;
  int m_rar_sent;

  int m_eab_index;

  bool m_eab_on;

  bool m_information_changed;

  int tt_pream;


  
};

#endif
