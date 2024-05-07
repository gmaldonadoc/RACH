/* 
 * File:   InformationManager.h
 * Author: Tiago Pedroso da Cruz de Andrade
 *
 */

#ifndef InformationManager_H
#define	InformationManager_H

#include <iostream>

static int PDCCHNumberOfCCEs[4] = {
  1, 2, 4, 8
};

static int PDCCHNumberOfBits[4] = {
  72, 144, 288, 576
};

static int PDCCHNumberOfCandidates_UESpecific[4] = {
  6, 6, 2, 2
};

static int PDCCHNumberOfCandidates_Common[4] = {
  -1, -1, 4, 2
};

class InformationManager {
public:
  InformationManager();
  ~InformationManager();

  int GetPDCCHNumberOfCCEs(int format);

  int GetPDCCHNumberOfCandidates_UESpecific(int format);

  int GetPDCCHNumberOfCandidates_Common(int format);

  static InformationManager* Init(void) {
    if (ptr == NULL) {
      ptr = new InformationManager();
    }
    return ptr;
  }

  int numberOfRA_Preambles; //4, 8, 12, 16, 2, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64
  int numberOfRA_Preambles_HTC; //4, 8, 12, 16, 2, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64

  int mac_ContentionResolutionTimer; //8, 16, 24, 32, 40, 48, 56, 64 subframes (ms))
  int preambleTransMax;
  int ra_ResponseWindowSize; //2, 3, 4, 5, 6, 7, 8, 10 subframes (ms)
  int ra_PRACH_MaskIndex;
  int maxHarq_Msg3Tx; // 1 - 8

  int m_pdcch_format; // 0 - 3

  int backoffIndicatorGeneral; // 0 - 15
  int backoffIndicatorHTC; // 0 - 15
  int backoffIndicatorMTC; // 0 - 15

  int ra_Method; //
  int harq_method; // 0 - Normal ; 1 - Probabilistic
  bool harq_method_god_mode; // allow the usage of real data from the eNB @UE
  int max_rar_decodable;

  bool acb_Dynamic;
  double ac_BarringFactor; // Access Barring Class / Barring Factor
  double ac_BarringTime; // Access Barring Class / Barring Time
  int acb_MonitoringPeriod; // re-calculated ACB-Dynamic

  bool noSensitive_Blocked;
  int QoSDracon_MonitoringPeriod; // periodo de tempo do ciclo de monitoramento do QoS-Dracon

  int ra_MinBE;
  int ra_MaxBE;

  int m_NumberOfCCEs;

  bool eab_Dynamic;
  bool eab_on;
  int eab_MonitoringPeriod; // periodo de alteracao do round robin do EAB
  int eab_BarringBitmap[10]; // Access Classes EAB Bitmap --> 0 - allow, 1 - barred

  int length_pagingCycle; // 32, 64, 128, 192, 256, 320, 384 radio frames
  int period_systemInformation; // 160, 320, 480, 640 ms

  int qtdPRBsTest;

  int numberOfGroups; // New Group RA
  int numberOfPreamblesPerGroup; // New Group RA

private:
  static InformationManager *ptr;
};

#endif
