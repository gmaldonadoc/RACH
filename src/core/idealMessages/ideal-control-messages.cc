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

#include "ideal-control-messages.h"
#include "../../device/NetworkNode.h"

#include "../../flows/application/WEB.h"
#include "../../componentManagers/FrameManager.h"

IdealControlMessage::IdealControlMessage(void) : m_source(0), m_destination(0)
{
  m_processingTime = 3;
  m_waitingTime = 0;
  m_collision = false;
  m_error = false;
  m_isQueue = false;
  m_msgID = m_msgIDGlobal++;
}

IdealControlMessage::~IdealControlMessage(void)
{
}

void IdealControlMessage::SetSourceDevice(NetworkNode *src)
{
  m_source = src;
}

void IdealControlMessage::SetDestinationDevice(NetworkNode *dst)
{
  m_destination = dst;
}

NetworkNode *
IdealControlMessage::GetSourceDevice(void)
{
  return m_source;
}

NetworkNode *
IdealControlMessage::GetDestinationDevice(void)
{
  return m_destination;
}

void IdealControlMessage::SetMessageType(IdealControlMessage::MessageType type)
{
  m_type = type;
}

IdealControlMessage::MessageType
IdealControlMessage::GetMessageType(void)
{
  return m_type;
}

void IdealControlMessage::setCollision(bool collision)
{
  m_collision = collision;
}

bool IdealControlMessage::getCollision()
{
  return m_collision;
}

void IdealControlMessage::setError(bool error)
{
  m_error = error;
}

bool IdealControlMessage::getError()
{
  return m_error;
}

int IdealControlMessage::getMsgID()
{
  return m_msgID;
}

bool IdealControlMessage::getIsQueue()
{
  return m_isQueue;
}

void IdealControlMessage::setIsQueue(bool condition)
{
  m_isQueue = condition;
}

// ----------------------------------------------------------------------------------------------------------

PdcchMapIdealControlMessage::PdcchMapIdealControlMessage(void)
{
  m_idealPdcchMessage = new IdealPdcchMessage();
  SetMessageType(IdealControlMessage::ALLOCATION_MAP);
}

PdcchMapIdealControlMessage::~PdcchMapIdealControlMessage(void)
{
  m_idealPdcchMessage->clear();
  delete m_idealPdcchMessage;
}

void PdcchMapIdealControlMessage::AddNewRecord(Direction direction, int subChannel, NetworkNode *ue, double mcs)
{
  IdealPdcchRecord record;
  record.m_direction = direction;
  record.m_idSubChannel = subChannel;
  record.m_mcsIndex = mcs;
  record.m_ue = ue;
  m_idealPdcchMessage->push_back(record);
}

PdcchMapIdealControlMessage::IdealPdcchMessage *
PdcchMapIdealControlMessage::GetMessage(void)
{
  return m_idealPdcchMessage;
}

// ----------------------------------------------------------------------------------------------------------

CqiIdealControlMessage::CqiIdealControlMessage(void)
{
  m_cqiFeedbacks = new CqiFeedbacks();
  SetMessageType(IdealControlMessage::CQI_FEEDBACKS);
}

CqiIdealControlMessage::~CqiIdealControlMessage(void)
{
  m_cqiFeedbacks->clear();
  delete m_cqiFeedbacks;
}

void CqiIdealControlMessage::AddNewRecord(int subChannel, double cqi, double sinr)
{
  CqiFeedback c;
  c.m_idSubChannel = subChannel;
  c.m_cqi = cqi;
  c.m_sinr = sinr;

  m_cqiFeedbacks->push_back(c);
}

CqiIdealControlMessage::CqiFeedbacks *
CqiIdealControlMessage::GetMessage(void)
{
  return m_cqiFeedbacks;
}

// ----------------------------------------------------------------------------------------------------------

ArqRlcIdealControlMessage::ArqRlcIdealControlMessage(void)
{
  SetMessageType(IdealControlMessage::ARQ_RLC_ACK);
}

ArqRlcIdealControlMessage::~ArqRlcIdealControlMessage(void)
{
}

void ArqRlcIdealControlMessage::SetAck(int ack)
{
  m_ack = ack;
}

int ArqRlcIdealControlMessage::GetAck(void)
{
  return m_ack;
}

void ArqRlcIdealControlMessage::SetStartByte(int b)
{
  m_startByte = b;
}

void ArqRlcIdealControlMessage::SetEndByte(int b)
{
  m_endByte = b;
}

int ArqRlcIdealControlMessage::GetStartByte(void)
{
  return m_startByte;
}

int ArqRlcIdealControlMessage::GetEndByte(void)
{
  return m_endByte;
}

// ----------------------------------------------------------------------------------------------------------

BufferStatusReportingIdealControlMessage::BufferStatusReportingIdealControlMessage(int triggerManager)
{
  SetMessageType(IdealControlMessage::BUFFER_STATUS_REPORTING);
  m_triggerManager = triggerManager;

  m_destinationUED2D = NULL;
}

BufferStatusReportingIdealControlMessage::~BufferStatusReportingIdealControlMessage()
{
}

int BufferStatusReportingIdealControlMessage::GetBufferStatusReport()
{
  return m_bufferStatusReport;
}

void BufferStatusReportingIdealControlMessage::SetBufferStatusReport(int bufferSize)
{
  m_bufferStatusReport = bufferSize;
}

int BufferStatusReportingIdealControlMessage::GetTriggerManager()
{
  return m_triggerManager;
}

void BufferStatusReportingIdealControlMessage::SetDestinationUeD2D(NetworkNode *UE)
{
  m_destinationUED2D = UE;
}

NetworkNode *
BufferStatusReportingIdealControlMessage::GetDestinationUeD2D()
{
  return m_destinationUED2D;
}

// ----------------------------------------------------------------------------------------------------------

RAPreambleIdealControlMessage::RAPreambleIdealControlMessage(int raRNTI, int preamble, int attempts)
{
  SetMessageType(IdealControlMessage::RA_PREAMBLE);
  m_raRNTI = raRNTI;
  m_preamble = preamble;

  nbAttempts = attempts;
}

RAPreambleIdealControlMessage::~RAPreambleIdealControlMessage(void)
{
}

int RAPreambleIdealControlMessage::GetPreamble()
{
  return m_preamble;
}

int RAPreambleIdealControlMessage::getRARNTI()
{
  return m_raRNTI;
}

int RAPreambleIdealControlMessage::getAttempts()
{
  return nbAttempts;
}

// ----------------------------------------------------------------------------------------------------------

RAResponseIdealControlMessage::RAResponseIdealControlMessage(int RARNTI, int backoffIndicator_general, int backoffIndicator_HTC, int backoffIndicator_MTC)
{
  SetMessageType(IdealControlMessage::RA_RESPONSE);
  m_idealRAResponse = new IdealRAResponse();
  m_raRNTI = RARNTI;
  m_backoffIndicator_general = backoffIndicator_general;
  m_backoffIndicator_HTC = backoffIndicator_HTC;
  m_backoffIndicator_MTC = backoffIndicator_MTC;
}

RAResponseIdealControlMessage::~RAResponseIdealControlMessage(void)
{
  m_idealRAResponse->clear();
  delete m_idealRAResponse;
}

int RAResponseIdealControlMessage::GetPreamble()
{
  return m_preamble;
}

void RAResponseIdealControlMessage::SetCondition(bool condition)
{
  m_isCorrect = condition;
}

bool RAResponseIdealControlMessage::GetCondition()
{
  return m_isCorrect;
}

double
RAResponseIdealControlMessage::GetSchedulingTime()
{
  return m_schedulingTime;
}

void RAResponseIdealControlMessage::SetBackoffIndicatorGeneral(int value)
{
  m_backoffIndicator_general = value;
}

int RAResponseIdealControlMessage::GetBackoffIndicatorGeneral()
{
  return m_backoffIndicator_general;
}

int RAResponseIdealControlMessage::GetBackoffIndicatorHTC()
{
  return m_backoffIndicator_HTC;
}

int RAResponseIdealControlMessage::GetBackoffIndicatorMTC()
{
  return m_backoffIndicator_MTC;
}

void RAResponseIdealControlMessage::UpdateRARSize(int size)
{
  m_RARSize == size;
}

void RAResponseIdealControlMessage::AddNewRecord(int preamble)
{
  IdealRARRecord record;
  record.m_preamble = preamble;
  m_idealRAResponse->push_back(record);
}

RAResponseIdealControlMessage::IdealRAResponse *
RAResponseIdealControlMessage::GetMessage(void)
{
  return m_idealRAResponse;
}

int RAResponseIdealControlMessage::getRARNTI()
{
  return m_raRNTI;
}

RAResponseIdealControlMessage *
RAResponseIdealControlMessage::Copy()
{
  RAResponseIdealControlMessage *newMsg = new RAResponseIdealControlMessage(this->getRARNTI(), this->GetBackoffIndicatorGeneral(), this->GetBackoffIndicatorHTC(), this->GetBackoffIndicatorMTC());
  newMsg->SetSourceDevice(this->GetSourceDevice());

  IdealRAResponse *rar = this->GetMessage();
  IdealRAResponse::iterator iter;

  for (iter = rar->begin(); iter != rar->end(); iter++)
  {
    newMsg->AddNewRecord((*iter).m_preamble);
  }

  return newMsg;
}

// ----------------------------------------------------------------------------------------------------------

ConnectionRequestIdealControlMessage::ConnectionRequestIdealControlMessage(int raRNTI, int resourceBlock, int overloadIndicator, bool sensitive)
{
  m_raRNTI = raRNTI;
  m_resourceBlock = resourceBlock;
  m_overload_indicator = overloadIndicator;
  m_sensitive = sensitive;
  m_respHARQ = false;
  m_fake_transmission = false;
  m_fake_ack = false;
  SetMessageType(IdealControlMessage::CONNECTION_REQUEST);
}

ConnectionRequestIdealControlMessage::~ConnectionRequestIdealControlMessage(void)
{
}

int ConnectionRequestIdealControlMessage::getResourceBlock()
{
  return m_resourceBlock;
}

int ConnectionRequestIdealControlMessage::getRARNTI()
{
  return m_raRNTI;
}

bool ConnectionRequestIdealControlMessage::getRespHARQ()
{
  return m_respHARQ;
}

void ConnectionRequestIdealControlMessage::setRespHARQ(bool value)
{
  m_respHARQ = value;
}

int ConnectionRequestIdealControlMessage::getOverloadIndicator()
{
  return m_overload_indicator;
}

bool ConnectionRequestIdealControlMessage::getSensitive()
{
  return m_sensitive;
}

void ConnectionRequestIdealControlMessage::SetHoLTime(double time)
{
  m_HoLTime = time;
}

void ConnectionRequestIdealControlMessage::SetMaxDelay(double delay)
{
  m_maxDelay = delay;
}

double
ConnectionRequestIdealControlMessage::GetHoLTime()
{
  return m_HoLTime;
}

double
ConnectionRequestIdealControlMessage::GetMaxDelay()
{
  return m_maxDelay;
}

void ConnectionRequestIdealControlMessage::setAsFakeTransmission()
{
  m_fake_transmission = true;
}

void ConnectionRequestIdealControlMessage::setAsRealTransmission()
{
  m_fake_transmission = false;
}

bool ConnectionRequestIdealControlMessage::isFakeTransmission()
{
  return m_fake_transmission;
}

void ConnectionRequestIdealControlMessage::setAsFakeAck()
{
  m_fake_ack = true;
}

bool ConnectionRequestIdealControlMessage::isFakeAck()
{
  return m_fake_ack;
}

// ----------------------------------------------------------------------------------------------------------

ContentionResolutionIdealControlMessage::ContentionResolutionIdealControlMessage(void)
{
  SetMessageType(IdealControlMessage::CONTENTION_RESOLUTION);
}

ContentionResolutionIdealControlMessage::~ContentionResolutionIdealControlMessage(void)
{
}

// ----------------------------------------------------------------------------------------------------------

HarqIdealControlMessage::HarqIdealControlMessage(int type, bool harqOfRandomAccessMsg3)
{
  SetMessageType(IdealControlMessage::HARQ);
  m_HARQType = type;
  m_harqOfRandomAccessMsg3 = harqOfRandomAccessMsg3;
  m_fake_ack = false;
  m_harq_cancelled = false;
}

HarqIdealControlMessage::~HarqIdealControlMessage(void)
{
}

int HarqIdealControlMessage::getHARQType()
{
  return m_HARQType;
}

bool HarqIdealControlMessage::GetHarqOfRandomAccessMsg3()
{
  return m_harqOfRandomAccessMsg3;
}

void HarqIdealControlMessage::SetHarqProcessId(int id)
{
  m_harqProcessId = id;
}

int HarqIdealControlMessage::GetHarqProcessId()
{
  return m_harqProcessId;
}

void HarqIdealControlMessage::setAsFakeAck()
{
  m_fake_ack = true;
}

bool HarqIdealControlMessage::isFakeAck()
{
  return m_fake_ack;
}

void HarqIdealControlMessage::cancelHARQ()
{
  m_harq_cancelled = true;
}

bool HarqIdealControlMessage::isHARQCancelled()
{
  return m_harq_cancelled;
}

// ----------------------------------------------------------------------------------------------------------

SchedulingRequestIdealControlMessage::SchedulingRequestIdealControlMessage(int raRNTI, int resourceBlock, int overloadIndicator, bool sensitive)
{
  m_raRNTI = raRNTI;
  m_resourceBlock = resourceBlock;
  m_overload_indicator = overloadIndicator;
  m_sensitive = sensitive;
  m_respHARQ = false;

  SetMessageType(IdealControlMessage::SCHEDULING_REQUEST);
}

SchedulingRequestIdealControlMessage::~SchedulingRequestIdealControlMessage(void)
{
}

int SchedulingRequestIdealControlMessage::getResourceBlock()
{
  return m_resourceBlock;
}

int SchedulingRequestIdealControlMessage::getRARNTI()
{
  return m_raRNTI;
}

bool SchedulingRequestIdealControlMessage::getRespHARQ()
{
  return m_respHARQ;
}

void SchedulingRequestIdealControlMessage::setRespHARQ(bool value)
{
  m_respHARQ = value;
}

int SchedulingRequestIdealControlMessage::getOverloadIndicator()
{
  return m_overload_indicator;
}

bool SchedulingRequestIdealControlMessage::getSensitive()
{
  return m_sensitive;
}

void SchedulingRequestIdealControlMessage::SetHoLTime(double time)
{
  m_HoLTime = time;
}

void SchedulingRequestIdealControlMessage::SetMaxDelay(double delay)
{
  m_maxDelay = delay;
}

double
SchedulingRequestIdealControlMessage::GetHoLTime()
{
  return m_HoLTime;
}

double
SchedulingRequestIdealControlMessage::GetMaxDelay()
{
  return m_maxDelay;
}

// ----------------------------------------------------------------------------------------------------------

SchedulingRequestPUCCHIdealControlMessage::SchedulingRequestPUCCHIdealControlMessage()
{
  SetMessageType(IdealControlMessage::SCHEDULING_REQUEST_PUCCH);
}

SchedulingRequestPUCCHIdealControlMessage::~SchedulingRequestPUCCHIdealControlMessage(void)
{
}

void SchedulingRequestPUCCHIdealControlMessage::SetHoLTime(double time)
{
  m_HoLTime = time;
}

void SchedulingRequestPUCCHIdealControlMessage::SetMaxDelay(double delay)
{
  m_maxDelay = delay;
}

double
SchedulingRequestPUCCHIdealControlMessage::GetHoLTime()
{
  return m_HoLTime;
}

double
SchedulingRequestPUCCHIdealControlMessage::GetMaxDelay()
{
  return m_maxDelay;
}

// ----------------------------------------------------------------------------------------------------------

BsrGrantIdealControlMessage::BsrGrantIdealControlMessage()
{
  SetMessageType(IdealControlMessage::BSR_GRANT);
}

BsrGrantIdealControlMessage::~BsrGrantIdealControlMessage(void)
{
}