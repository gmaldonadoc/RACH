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


#ifndef ENB_LTE_PHY_H_
#define ENB_LTE_PHY_H_

#include "lte-phy.h"

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

class EnbLtePhy : public LtePhy {
public:
  EnbLtePhy();
  virtual ~EnbLtePhy();

  virtual void DoSetBandwidthManager(void);

  virtual void StartTx(PacketBurst* p);
  virtual void StartRx(PacketBurst* p, TransmittedSignal* txSignal);
  virtual void EndRx(PacketBurst *p, TransmittedSignal *txSignal);

  virtual void SendIdealControlMessage(IdealControlMessage *msg);
  virtual void ReceiveIdealControlMessage(IdealControlMessage *msg, NetworkNode *src, TransmittedSignal *txSignal);

  void ReceiveReferenceSymbols(NetworkNode* n, TransmittedSignal* s);

  void SendReferenceSymbols(void);
  void SetTxSignalForReferenceSymbols(void);
  TransmittedSignal* GetTxSignalForReferenceSymbols(void);

  void ResetNoiseInterference(void);

  virtual void ReceiveSoundingReferenceSignal(NetworkNode *n, TransmittedSignal *s);

private:
  TransmittedSignal *m_txSignalForRerferenceSymbols;

  double m_noiseInterference[100]; // in W
};

#endif
