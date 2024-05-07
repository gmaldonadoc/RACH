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

#ifndef MAC_ENTITY_H
#define MAC_ENTITY_H

#include <list>

static int BackoffParameterValue[16] = {0, 10, 20, 30, 40, 60, 80, 120, 160, 240, 320, 480, 960, 0, 0, 0};

class Packet;
class NetworkNode;
class AMCModule;
class HarqManager;

/*
 * This class provides a basic implementation of the MAC layer
 */
class MacEntity {
public:

  MacEntity(void);
  virtual ~MacEntity(void);

  void Destroy(void);

  void SetDevice(NetworkNode* d);
  NetworkNode* GetDevice();

  void SetAmcModule(AMCModule* amcModule);
  AMCModule* GetAmcModule(void) const;

  HarqManager* GetHarqManager(void);
  
  void SetQCI(int QCINumber, int QCIPriority, int QCIDelay, bool QCISensitive, int QCIBackoffExponent);
  int GetQCINumber(void);
  int GetQCIPriority(void);
  int GetQCIBackoffExponent(void);
  bool GetQCISensitive(void);
  
  virtual int GetRealNbOfRBsForULScheduling(void);
  
  virtual int getCurrentPDCCHCCEs(void);
  virtual void updateCurrentPDCCHCCEs(int cces);

private:
  NetworkNode* m_device;
  AMCModule* m_amcModule;
  HarqManager* m_harqmanager;
  
  int m_qci_number;
  int m_qci_priority;
  int m_qci_delay; // ms
  bool m_qci_sensitive; // true - sensitive, false - tolerant
  int m_qci_backoff_exponent;
};


#endif /* MAC_ENTITY_H */
