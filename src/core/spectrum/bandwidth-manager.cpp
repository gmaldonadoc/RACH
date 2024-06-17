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

#include "bandwidth-manager.h"
#include <cstdio>
#include <iostream>
#include <cmath>

#define UL_LOW_FREQUENCY_BAND_1 1920 	//MHz
#define UL_HIGH_FREQUENCY_BAND_1 1980 	//MHz
#define DL_LOW_FREQUENCY_BAND_1 2110 	//MHz
#define DL_HIGH_FREQUENCY_BAND_1 2170 	//MHz

#define RBs_FOR_1_4_MHz 6
#define RBs_FOR_3_MHz 15
#define RBs_FOR_5_MHz 25
#define RBs_FOR_10_MHz 50
#define RBs_FOR_15_MHz 75
#define RBs_FOR_20_MHz 100

// Tira os PRBs para controle (PUCCH)
#define Ul_RBs_FOR_1_4_MHz 5
#define Ul_RBs_FOR_3_MHz 13
#define Ul_RBs_FOR_5_MHz 21
#define Ul_RBs_FOR_10_MHz 42
#define Ul_RBs_FOR_15_MHz 73
#define Ul_RBs_FOR_20_MHz 84

#define PUCCH_FOR_1_4_MHz 1
#define PUCCH_FOR_3_MHz 2
#define PUCCH_FOR_5_MHz 4
#define PUCCH_FOR_10_MHz 8
#define PUCCH_FOR_15_MHz 12
#define PUCCH_FOR_20_MHz 16

BandwidthManager::BandwidthManager() {
}

BandwidthManager::BandwidthManager(double ulBw, double dlBw, int ulOffset, int dlOffset) {
  m_ulBandwidth = ulBw;
  m_dlBandwidth = dlBw;
  m_ulOffsetBw = ulOffset;
  m_dlOffsetBw = dlOffset;

  m_operativeSubBand = 1;

  m_dlCFI = 3;

  m_dlSubChannels.clear();
  m_ulSubChannels.clear();

  if (dlBw == 1.4) {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_1_4_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
  } else if (dlBw == 3) {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_3_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
  } else if (dlBw == 5) {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_5_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
    switch (m_dlCFI) {
      case 1:
        m_pdcch_total_cces = 3;
        break;
      case 2:
        m_pdcch_total_cces = 12;
        break;
      case 3:
        m_pdcch_total_cces = 20;
        break;
    }
  } else if (dlBw == 10) {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_10_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
    switch (m_dlCFI) {
      case 1:
        m_pdcch_total_cces = 8;
        break;
      case 2:
        m_pdcch_total_cces = 25;
        break;
      case 3:
        m_pdcch_total_cces = 41;
        break;
    }
  } else if (dlBw == 15) {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_15_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
  } else if (dlBw == 20) {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_20_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
    switch (m_dlCFI) {
      case 1:
        m_pdcch_total_cces = 17;
        break;
      case 2:
        m_pdcch_total_cces = 50;
        break;
      case 3:
        m_pdcch_total_cces = 84;
        break;
    }
  }

  if (ulBw == 1.4) {
    for (int i = ulOffset; i < ulOffset + Ul_RBs_FOR_1_4_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
    m_pucchPRBs = PUCCH_FOR_1_4_MHz;
  } else if (ulBw == 3) {
    for (int i = ulOffset; i < ulOffset + Ul_RBs_FOR_3_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
    m_pucchPRBs = PUCCH_FOR_3_MHz;
  } else if (ulBw == 5) {
    for (int i = ulOffset; i < ulOffset + Ul_RBs_FOR_5_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
    m_pucchPRBs = PUCCH_FOR_5_MHz;
  } else if (ulBw == 10) {
    for (int i = ulOffset; i < ulOffset + Ul_RBs_FOR_10_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
    m_pucchPRBs = PUCCH_FOR_10_MHz;
  } else if (ulBw == 15) {
    for (int i = ulOffset; i < ulOffset + Ul_RBs_FOR_15_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
    m_pucchPRBs = PUCCH_FOR_15_MHz;
  } else if (ulBw == 20) {
    for (int i = ulOffset; i < ulOffset + Ul_RBs_FOR_20_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
    m_pucchPRBs = PUCCH_FOR_20_MHz;
  } else {
    for (int i = ulOffset; i < ulOffset + Ul_RBs_FOR_5_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
    m_pucchPRBs = PUCCH_FOR_5_MHz;
  }
}

BandwidthManager::BandwidthManager(double ulBw, double dlBw, int ulOffset, int dlOffset, bool tddTrue) {
  m_ulBandwidth = ulBw + dlBw;
  m_dlBandwidth = dlBw + ulBw;
  m_ulOffsetBw = ulOffset;
  m_dlOffsetBw = dlOffset;

  m_operativeSubBand = 1;

  m_dlSubChannels.clear();
  m_ulSubChannels.clear();

  if (dlBw == 1.4) {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_1_4_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
  } else if (dlBw == 3) {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_3_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
  } else if (dlBw == 5) {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_5_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
  } else if (dlBw == 10) {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_10_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
  } else if (dlBw == 15) {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_15_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
  } else if (dlBw == 20) {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_20_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
  } else {
    for (int i = dlOffset; i < dlOffset + RBs_FOR_5_MHz; i++) {
      m_dlSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_ulSubChannels.push_back(DL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
  }

  if (ulBw == 1.4) {
    for (int i = ulOffset; i < ulOffset + RBs_FOR_1_4_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
  } else if (ulBw == 3) {
    for (int i = ulOffset; i < ulOffset + RBs_FOR_3_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
  } else if (ulBw == 5) {
    for (int i = ulOffset; i < ulOffset + RBs_FOR_5_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
  } else if (ulBw == 10) {
    for (int i = ulOffset; i < ulOffset + RBs_FOR_10_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
  } else if (ulBw == 15) {
    for (int i = ulOffset; i < ulOffset + RBs_FOR_15_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
  } else if (ulBw == 20) {
    for (int i = ulOffset; i < ulOffset + RBs_FOR_20_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
  } else {
    for (int i = ulOffset; i < ulOffset + RBs_FOR_5_MHz; i++) {
      m_ulSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
      m_dlSubChannels.push_back(UL_LOW_FREQUENCY_BAND_1 + (i * 0.18));
    }
  }
}

BandwidthManager::~BandwidthManager() {
}

void
BandwidthManager::SetDlSubChannels(std::vector<double> s) {
  m_dlSubChannels = s;
}

std::vector<double>
BandwidthManager::GetDlSubChannels(void) {
  return m_dlSubChannels;
}

void
BandwidthManager::SetUlSubChannels(std::vector<double> s) {
  m_ulSubChannels = s;
}

std::vector<double>
BandwidthManager::GetUlSubChannels(void) {
  return m_ulSubChannels;
}

void
BandwidthManager::SetOperativeSubBand(int s) {
  m_operativeSubBand = s;
}

int
BandwidthManager::GetOperativeSubBand(void) {
  return m_operativeSubBand;
}

void
BandwidthManager::SetUlBandwidth(double b) {
  m_ulBandwidth = b;
}

void
BandwidthManager::SetDlBandwidth(double b) {
  m_dlBandwidth = b;
}

void
BandwidthManager::SetUlOffsetBw(int o) {
  m_ulOffsetBw = o;
}

void
BandwidthManager::SetDlOffsetBw(int o) {
  m_dlOffsetBw = o;
}

double
BandwidthManager::GetUlBandwidth(void) {
  return m_ulBandwidth;
}

double
BandwidthManager::GetDlBandwidth(void) {
  return m_dlBandwidth;
}

int
BandwidthManager::GetUlOffsetBw(void) {
  return m_ulOffsetBw;
}

int
BandwidthManager::GetDlOffsetBw(void) {
  return m_dlOffsetBw;
}

int
BandwidthManager::GetPDCCHTotalCces(void) {
  return m_pdcch_total_cces;
}

int
BandwidthManager::GetPUCCHTotalPrbs(void) {
  return m_pucchPRBs;
}

BandwidthManager*
BandwidthManager::Copy() {
  BandwidthManager *s = new BandwidthManager();
  s->SetDlBandwidth(GetDlBandwidth());
  s->SetUlBandwidth(GetUlBandwidth());
  s->SetDlOffsetBw(GetDlOffsetBw());
  s->SetUlOffsetBw(GetUlOffsetBw());
  s->SetDlSubChannels(GetDlSubChannels());
  s->SetUlSubChannels(GetUlSubChannels());
  s->SetOperativeSubBand(GetOperativeSubBand());

  return s;
}

void
BandwidthManager::Print(void) {
  std::cout << "BandwidthManager: " << this << std::endl;

  std::cout << "\t operative sub band: " << m_operativeSubBand <<
          "\n\t m_dlBandwidth " << m_dlBandwidth <<
          "\n\t m_ulBandwidth " << m_ulBandwidth <<
          "\n\t m_dlOffsetBw " << m_dlOffsetBw <<
          "\n\t m_ulOffsetBw " << m_ulOffsetBw << std::endl;

  std::cout << "\t DL channels: ";
  for (int i = 0; i < m_dlSubChannels.size(); i++) {
    std::cout << m_dlSubChannels.at(i) << " ";
  }
  std::cout << std::endl;

  std::cout << "\t UL channels: ";
  for (int i = 0; i < m_ulSubChannels.size(); i++) {
    std::cout << m_ulSubChannels.at(i) << " ";
  }
  std::cout << std::endl;
}