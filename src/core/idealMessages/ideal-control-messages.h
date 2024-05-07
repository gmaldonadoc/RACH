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


#ifndef IDEAL_CONTROL_MESSAGES_H
#define IDEAL_CONTROL_MESSAGES_H

#include <list>

class NetworkNode;

/*
 * The IdealControlMessage provides a basic implementations for
 * control messages (such as PDCCH allocation map, CQI feedbacks)
 * that are exchanged among eNodeB and UEs.
 */
class IdealControlMessage {
public:

  enum MessageType {
    CQI_FEEDBACKS,
    ALLOCATION_MAP,
    ARQ_RLC_ACK,
    SCHEDULING_REQUEST,
    RA_PREAMBLE,
    RA_RESPONSE,
    CONNECTION_REQUEST,
    CONTENTION_RESOLUTION,
    HARQ,
    BUFFER_STATUS_REPORTING,
    UL_RESOURCE_ALLOCATION,
    SCHEDULING_REQUEST_PUCCH,
    BSR_GRANT
  };

  IdealControlMessage(void);
  virtual ~IdealControlMessage(void);

  void SetSourceDevice(NetworkNode *src);
  void SetDestinationDevice(NetworkNode *dst);

  NetworkNode* GetSourceDevice(void);
  NetworkNode* GetDestinationDevice(void);

  void SetMessageType(MessageType type);
  MessageType GetMessageType(void);

  void setCollision(bool collision);
  bool getCollision(void);

  void setError(bool error);
  bool getError(void);

  int getMsgID(void);

  bool getIsQueue(void);
  void setIsQueue(bool condition);

  int m_processingTime;
  int m_waitingTime;
private:
  NetworkNode *m_source;
  NetworkNode *m_destination;
  MessageType m_type;

  int m_msgID;
  int m_msgIDGlobal;

  bool m_collision;
  bool m_error;

  bool m_isQueue;
};

#endif /* IDEAL_CONTROL_MESSAGES_H */

// ----------------------------------------------------------------------------------------------------------

#ifndef PDCCH_MAP_IDEAL_CONTROL_MESSAGES_H
#define PDCCH_MAP_IDEAL_CONTROL_MESSAGES_H

#include <list>

class NetworkNode;

/*
 * The PdcchMapIdealControlMessage defines an ideal allocation map
 * for both UL and DL sends by the eNodeB to all UE,
 * using an ideal PDCCH control channel.
 * IdealPdcchMessage is composed by a list of IdealPdcchRecord
 * where is indicated the UE that can use a particular sub channel
 * with a proper MCS scheme.
 *
 * This records are the same for both UL and DL, and are created by the
 * packet scheduler at the beginning of each sub frame.
 *
 * When the IdealPdcchMessage is sent under an ideal control channel,
 * all UE stores into a proper variables the informations about
 * the resource mapping.
 */
class PdcchMapIdealControlMessage : public IdealControlMessage {
public:

  PdcchMapIdealControlMessage(void);
  virtual ~PdcchMapIdealControlMessage(void);

  enum Direction {
    DOWNLINK,
    UPLINK
  };

  struct IdealPdcchRecord {
    Direction m_direction;
    NetworkNode *m_ue;
    int m_idSubChannel;
    double m_mcsIndex;
  };

  typedef std::list<struct IdealPdcchRecord> IdealPdcchMessage;

  void AddNewRecord(Direction direction, int subChannel, NetworkNode* ue, double mcs);

  IdealPdcchMessage* GetMessage(void);

private:
  IdealPdcchMessage *m_idealPdcchMessage;
};

#endif

// ----------------------------------------------------------------------------------------------------------

#ifndef CQI_IDEAL_CONTROL_MESSAGES_H
#define CQI_IDEAL_CONTROL_MESSAGES_H

#include <list>

class NetworkNode;

/*
 * The CqiIdealControlMessage defines an ideal list of feedback about
 * the channel quality sent by the UE to the eNodeB.
 */
class CqiIdealControlMessage : public IdealControlMessage {
public:
  CqiIdealControlMessage(void);
  virtual ~CqiIdealControlMessage(void);

  struct CqiFeedback {
    int m_idSubChannel;
    double m_cqi;
    double m_sinr;
  };

  typedef std::list<struct CqiFeedback> CqiFeedbacks;

  void AddNewRecord(int subChannel, double cqi, double sinr);
  CqiFeedbacks* GetMessage(void);

private:
  CqiFeedbacks *m_cqiFeedbacks;

};

#endif /* CQI_IDEAL_CONTROL_MESSAGES_H */

// ----------------------------------------------------------------------------------------------------------

#ifndef ARQ_IDEAL_CONTROL_MESSAGES_H
#define ARQ_IDEAL_CONTROL_MESSAGES_H

#include <list>

class NetworkNode;

class ArqRlcIdealControlMessage : public IdealControlMessage {
public:

  ArqRlcIdealControlMessage(void);
  virtual ~ArqRlcIdealControlMessage(void);

  void SetAck(int ack);
  int GetAck(void);

  void SetStartByte(int b);
  void SetEndByte(int b);
  int GetStartByte(void);
  int GetEndByte(void);

private:
  int m_ack;
  int m_startByte;
  int m_endByte;
};

#endif

// ----------------------------------------------------------------------------------------------------------

#ifndef BUFFER_STATUS_REPORTING_IDEAL_CONTROL_MESSAGES_H
#define BUFFER_STATUS_REPORTING_IDEAL_CONTROL_MESSAGES_H

#include <list>

class NetworkNode;

class BufferStatusReportingIdealControlMessage : public IdealControlMessage {
public:

  BufferStatusReportingIdealControlMessage(int triggerManager);
  virtual ~BufferStatusReportingIdealControlMessage(void);

  int GetBufferStatusReport(void);
  void SetBufferStatusReport(int bufferSize);

  int GetTriggerManager(void);

  void SetDestinationUeD2D(NetworkNode *UE);
  NetworkNode* GetDestinationUeD2D(void);

private:

  int m_triggerManager; // 1 - REGULAR, 2 - PADDING and 3 - PERIODIC
  int m_bufferStatusReport;
  NetworkNode *m_destinationUED2D;
};

#endif

// ----------------------------------------------------------------------------------------------------------

#ifndef RA_PREAMBLE_IDEAL_CONTROL_MESSAGES_H
#define RA_PREAMBLE_IDEAL_CONTROL_MESSAGES_H

#include <list>

class NetworkNode;

class RAPreambleIdealControlMessage : public IdealControlMessage {
public:
  RAPreambleIdealControlMessage(int raRNTI, int preamble, int attempts);

  virtual ~RAPreambleIdealControlMessage(void);

  int GetPreamble();

  int getRARNTI();

  int getAttempts();

private:
  int m_raRNTI;
  int m_preamble;

  int nbAttempts;
};

#endif /* RA_PREAMBLE_IDEAL_CONTROL_MESSAGES_H */

// ----------------------------------------------------------------------------------------------------------

#ifndef RA_RESPONSE_IDEAL_CONTROL_MESSAGES_H
#define RA_RESPONSE_IDEAL_CONTROL_MESSAGES_H

#include <list>

class NetworkNode;

class RAResponseIdealControlMessage : public IdealControlMessage {
public:
  RAResponseIdealControlMessage(int RARNTI, int backoffIndicator_general, int backoffIndicator_HTC, int backoffIndicator_MTC);
  virtual ~RAResponseIdealControlMessage(void);

  struct IdealRARRecord {
    int m_preamble;
  };

  int GetPreamble(void);
  void SetCondition(bool value);
  double GetSchedulingTime(void);
  bool GetCondition(void);

  void SetBackoffIndicatorGeneral(int value);
  int GetBackoffIndicatorGeneral(void);
  int GetBackoffIndicatorHTC(void);
  int GetBackoffIndicatorMTC(void);

  void UpdateRARSize(int size);

  int getRARNTI();

  typedef std::list<struct IdealRARRecord> IdealRAResponse;

  void AddNewRecord(int preamble);

  IdealRAResponse* GetMessage(void);

  RAResponseIdealControlMessage* Copy();

private:
  // Random Access Preamble Identifies
  int m_preamble;
  int m_raRNTI;

  // Backoff Indicator
  int m_backoffIndicator_general;
  int m_backoffIndicator_HTC;
  int m_backoffIndicator_MTC;

  bool m_isCorrect;
  double m_schedulingTime;
  int m_RARSize;

  IdealRAResponse *m_idealRAResponse;
};

#endif /* RA_RESPONSE_IDEAL_CONTROL_MESSAGES_H */

// ----------------------------------------------------------------------------------------------------------

#ifndef CONNECTION_REQUEST_IDEAL_CONTROL_MESSAGES_H
#define CONNECTION_REQUEST_IDEAL_CONTROL_MESSAGES_H

#include <list>

class NetworkNode;

class ConnectionRequestIdealControlMessage : public IdealControlMessage {
public:

  ConnectionRequestIdealControlMessage(int raRNTI, int resourceBlock, int overloadIndicator, bool sensitive);

  virtual ~ConnectionRequestIdealControlMessage(void);

  int getResourceBlock(void);

  int getRARNTI(void);

  void setRespHARQ(bool value);
  bool getRespHARQ(void);

  void setAsFakeTransmission(void); // Function used to bypass harq for harq_method = 1
  void setAsRealTransmission(void); // Function used to bypass harq for harq_method = 1
  bool isFakeTransmission(void);    // Function used to bypass harq for harq_method = 1

  void setAsFakeAck(void); // Function used to bypass harq for harq_method = 1
  bool isFakeAck(void);    // Function used to bypass harq for harq_method = 1

  // Usado pelo QoS-DRACON
  int getOverloadIndicator(void);
  bool getSensitive(void);
  // ---------------

  void SetHoLTime(double time);
  double GetHoLTime(void);
  void SetMaxDelay(double delay);
  double GetMaxDelay(void);

private:
  int m_resourceBlock;
  int m_raRNTI;
  bool m_respHARQ;

  bool m_fake_transmission; // Variable used to bypass harq for harq_method = 1
  bool m_fake_ack;          // Variable used to bypass harq for harq_method = 1

  // Usado pelo QoS-DRACON
  int m_overload_indicator;
  bool m_sensitive;
  // -------------

  double m_HoLTime;
  double m_maxDelay;
};

#endif

// ----------------------------------------------------------------------------------------------------------

#ifndef CONTENTION_RESOLUTION_IDEAL_CONTROL_MESSAGES_H
#define CONTENTION_RESOLUTION_IDEAL_CONTROL_MESSAGES_H

#include <list>

class NetworkNode;

class ContentionResolutionIdealControlMessage : public IdealControlMessage {
public:

  ContentionResolutionIdealControlMessage(void);

  virtual ~ContentionResolutionIdealControlMessage(void);

private:
};

#endif

// ----------------------------------------------------------------------------------------------------------

#ifndef HARQ_IDEAL_CONTROL_MESSAGES_H
#define HARQ_IDEAL_CONTROL_MESSAGES_H

#include <list>

class NetworkNode;

class HarqIdealControlMessage : public IdealControlMessage {
public:

  HarqIdealControlMessage(int type, bool harqOfRandomAccessMsg3);
  virtual ~HarqIdealControlMessage(void);

  void SetHarqProcessId(int id);
  int GetHarqProcessId(void);

  int getHARQType(void);

  bool GetHarqOfRandomAccessMsg3(void);

  void setAsFakeAck(void);
  bool isFakeAck(void);

  void cancelHARQ(void);
  bool isHARQCancelled(void);


private:
  int m_HARQType; // 1 - ACK; 2 - NACK
  bool m_harqOfRandomAccessMsg3;
  int m_harqProcessId;
  bool m_fake_ack;
  bool m_harq_cancelled;    // Variable used by enb to cancel harq for harq_method = 1
};

#endif

// ----------------------------------------------------------------------------------------------------------

#ifndef SCHEDULING_REQUEST_IDEAL_CONTROL_MESSAGES_H
#define SCHEDULING_REQUEST_IDEAL_CONTROL_MESSAGES_H

#include <list>

class NetworkNode;

class SchedulingRequestIdealControlMessage : public IdealControlMessage {
public:

  SchedulingRequestIdealControlMessage(int raRNTI, int resourceBlock, int overloadIndicator, bool sensitive);

  virtual ~SchedulingRequestIdealControlMessage(void);

  int getResourceBlock(void);

  int getRARNTI(void);

  void setRespHARQ(bool value);
  bool getRespHARQ(void);

  // Usado pelo QoS-DRACON
  int getOverloadIndicator(void);
  bool getSensitive(void);
  // ---------------

  void SetHoLTime(double time);
  double GetHoLTime(void);
  void SetMaxDelay(double delay);
  double GetMaxDelay(void);

private:
  int m_resourceBlock;
  int m_raRNTI;
  bool m_respHARQ;

  // Usado pelo QoS-DRACON
  int m_overload_indicator;
  bool m_sensitive;
  // -------------

  double m_HoLTime;
  double m_maxDelay;
};

#endif

// ----------------------------------------------------------------------------------------------------------

#ifndef SCHEDULING_REQUEST_PUCCH_IDEAL_CONTROL_MESSAGES_H
#define SCHEDULING_REQUEST_PUCCH_IDEAL_CONTROL_MESSAGES_H

class NetworkNode;

class SchedulingRequestPUCCHIdealControlMessage : public IdealControlMessage {
public:
  SchedulingRequestPUCCHIdealControlMessage();
  virtual ~SchedulingRequestPUCCHIdealControlMessage(void);

  void SetHoLTime(double time);
  double GetHoLTime(void);

  void SetMaxDelay(double delay);
  double GetMaxDelay(void);

private:
  double m_HoLTime;
  double m_maxDelay;
};

#endif

// ----------------------------------------------------------------------------------------------------------

#ifndef BSR_GRANT_IDEAL_CONTROL_MESSAGES_H
#define BSR_GRANT_IDEAL_CONTROL_MESSAGES_H

class NetworkNode;

class BsrGrantIdealControlMessage : public IdealControlMessage {
public:
  BsrGrantIdealControlMessage();
  virtual ~BsrGrantIdealControlMessage(void);

private:
};

#endif
