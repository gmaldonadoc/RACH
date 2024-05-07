/* 
 * File:   InformationManager.cpp
 * Author: Tiago Pedroso da Cruz de Andrade
 */

#include "InformationManager.h"

InformationManager *InformationManager::ptr = NULL;

InformationManager::InformationManager() {
  numberOfRA_Preambles = 52;
  mac_ContentionResolutionTimer = 48;
  preambleTransMax = 10;
  ra_ResponseWindowSize = 5;
  ra_PRACH_MaskIndex = 6;
  maxHarq_Msg3Tx = 4;

  backoffIndicatorGeneral = 2;

  m_NumberOfCCEs = 16; // quantidade de recursos no PDCCH quando somente o RA é simulado

  m_pdcch_format = 0;

  // SB
  backoffIndicatorHTC = 2;
  backoffIndicatorMTC = 2;

  // ACB
  acb_Dynamic = false;
  ac_BarringFactor = 1;
  ac_BarringTime = 4;
  acb_MonitoringPeriod = 75; // in milliseconds

  // SAB
  ra_MinBE = 2;
  ra_MaxBE = 8;

  // RRS / Hybrid
  numberOfRA_Preambles_HTC = 0;

  // QoS-Dracon
  QoSDracon_MonitoringPeriod = 50; // in milliseconds
  noSensitive_Blocked = false;

  // EAB
  eab_Dynamic = false;
  eab_on = false;
  eab_MonitoringPeriod = 500; // in milliseconds
  eab_BarringBitmap[0] = 0;
  eab_BarringBitmap[1] = 0;
  eab_BarringBitmap[2] = 0;
  eab_BarringBitmap[3] = 0;
  eab_BarringBitmap[4] = 0;
  eab_BarringBitmap[5] = 0;
  eab_BarringBitmap[6] = 0;
  eab_BarringBitmap[7] = 0;
  eab_BarringBitmap[8] = 0;
  eab_BarringBitmap[9] = 0;

  ra_Method = 1;
  harq_method = 1;
  harq_method_god_mode = false;
  max_rar_decodable = numberOfRA_Preambles < 15? numberOfRA_Preambles:15;

  // System Information
  length_pagingCycle = 128;
  period_systemInformation = 500; // in milliseconds
}

InformationManager::~InformationManager() {
}

int
InformationManager::GetPDCCHNumberOfCCEs(int format) {
  return PDCCHNumberOfCCEs[format];
}

int
InformationManager::GetPDCCHNumberOfCandidates_UESpecific(int format) {
  return PDCCHNumberOfCandidates_UESpecific[format];
}

int
InformationManager::GetPDCCHNumberOfCandidates_Common(int format) {
  return PDCCHNumberOfCandidates_Common[format];
}