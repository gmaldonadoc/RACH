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

/*
 * LTE-Sim is the main program of the LTE-Sim simulator.
 * To run simulations you can
 * (i) select one of example scenarios developed in Scenario/ ("./LTE-Sim -h" for details)
 * (ii) create a new scenario and add its reference into the main program.
 *
 *  To create a new scenario, see documentation in DOC/ folder.
 *
 *  For any questions, please contact the author at
 *  g.piro@poliba.it
 */

#include "TEST/test.h"

#include <iostream>
#include <queue>
#include <fstream>
#include <stdlib.h>
#include <cstring>

#include "utility/help.h"

#include "scenarios/scenario-01.h"
#include "scenarios/scenario-02.h"
#include "scenarios/scenario-03.h"
#include "scenarios/scenario-04.h"
#include "scenarios/scenario-05.h"
#include "scenarios/scenario-06.h"
#include "scenarios/scenario-07.h"
#include "scenarios/scenario-08.h"
#include "scenarios/scenario-09.h"
#include "scenarios/scenario-10.h"
#include "scenarios/scenario-11.h"
#include "scenarios/scenario-12.h"
#include "scenarios/scenario-13.h"
#include "scenarios/scenario-14.h"

#include "scenarios/d2d-scenario.h"

int
main(int argc, char *argv[]) {

  if (argc > 1) {

    /* Help */
    if (!strcmp(argv[1], "-h") || !strcmp(argv[1], "-H") || !strcmp(argv[1], "--help") || !strcmp(argv[1], "--Help")) {
      Help();
      return 0;
    }

    /* run tests */
    if (strcmp(argv[1], "test") == 0) {
    }

    /* My scenarios */
    if (strcmp(argv[1], "scenario-01") == 0) {
      int raMethod = atoi(argv[2]); // RA scheme
      int seed = atoi(argv[3]); // semente da simulacao
      int nbUE_HTC = atoi(argv[4]); // numero de dispositivos HTC
      int nbUE_MTC = atoi(argv[5]); // numero de dispositivos MTC
      int RACHIndex = atoi(argv[6]); // configuracao da oportunidade RACH
      int preambleTransMax = atoi(argv[7]); // maximo de transmissoes do preamble
      int backoffIndexGeneral = atoi(argv[8]); // backoff indicator geral
      int backoffIndexHTC = atoi(argv[9]); // backoff indicator para HTC quando usando o Specific Backoff
      int backoffIndexMTC = atoi(argv[10]); // backoff indicator para MTC quando usando o Specific Backoff
      int nbTotalPreambles = atoi(argv[11]); // quantidade total de preambles para contention-based RA
      int nbHTCPreambles = atoi(argv[12]); // quantidade de preambles destinados para HTC quando usando o RRS
      double acbBF = atof(argv[13]); // Access Barring Factor do ACB
      double acbBT = atof(argv[14]); // Access Barring Time do ACB - in seconds
      Scenario_01(raMethod, seed, nbUE_HTC, nbUE_MTC, RACHIndex, preambleTransMax, backoffIndexGeneral, backoffIndexHTC, backoffIndexMTC, nbTotalPreambles, nbHTCPreambles, acbBF, acbBT);
    }

    if (strcmp(argv[1], "scenario-02") == 0) {
      int seed = atoi(argv[2]); // seed of the simulation
      int raMethod = atoi(argv[3]); // RAN Overload Control Scheme
      double radius = atof(argv[4]); // coverage radius
      int ulBW = atoi(argv[5]); // uplink bandwidth
      int dlBW = atoi(argv[6]); // downlink bandwidth
      double acbBarringFactor = atof(argv[7]);
      double acbBarringTime = atof(argv[8]);
      int nHTC = atoi(argv[9]);
      int nMTC = atoi(argv[10]);
      Scenario_02(seed, raMethod, radius, ulBW, dlBW, acbBarringFactor, acbBarringTime, nHTC, nMTC);
    }

    if (strcmp(argv[1], "scenario-03") == 0) {
      int raMethod = atoi(argv[2]);
      int seed = atoi(argv[3]);
      int nbUE_H2H = atoi(argv[4]);
      int nbUE_M2M = atoi(argv[5]);
      int RACHIndex = atoi(argv[6]);
      int preambleTransMax = atoi(argv[7]);
      int RARWindowSize = atoi(argv[8]);
      int backoffIndex = atoi(argv[9]);
      int nbPreambles = atoi(argv[10]);
      double acbBF = atof(argv[11]); // Access Barring Factor
      double acbBT = atof(argv[12]); // Access Barring Time
      Scenario_03(raMethod, seed, nbUE_H2H, nbUE_M2M, RACHIndex, preambleTransMax, RARWindowSize, backoffIndex, nbPreambles, acbBF, acbBT);
    }

    if (strcmp(argv[1], "scenario-04") == 0) {
      int seed = atoi(argv[2]);
      int raMethod = atof(argv[3]); // RAN Overload Control Scheme
      double radius = atof(argv[4]); // Km
      int ulBW = atoi(argv[5]); // uplink bandwidth (MHz)
      int dlBW = atoi(argv[6]); // downlink bandwidth (MHz)
      int nHTC = atoi(argv[7]); // number of HTC devices
      int nMTC = atoi(argv[8]); // number of MTC devices
      int useHandover = atoi(argv[9]); // 0 -> false or 1 -> true
      int pdcchSch = atoi(argv[10]);
      Scenario_04(seed, raMethod, radius, ulBW, dlBW, nHTC, nMTC, useHandover, pdcchSch);
    }

    if (strcmp(argv[1], "scenario-05") == 0) {
      int seed = atoi(argv[2]);
      int raMethod = atof(argv[3]);
      double radius = atof(argv[4]); // Km
      int ulBW = atoi(argv[5]); // MHz
      int dlBW = atoi(argv[6]); // MHz
      int nUE_HTC_VoIP = atoi(argv[7]);
      int nUE_HTC_Video = atoi(argv[8]);
      int nUE_HTC_CBR = atoi(argv[9]);
      int nUE_MTC_TimeDriven = atoi(argv[10]);
      Scenario_05(seed, raMethod, radius, ulBW, dlBW, nUE_HTC_VoIP, nUE_HTC_Video, nUE_HTC_CBR, nUE_MTC_TimeDriven);
    }

    if (strcmp(argv[1], "scenario-06") == 0) {
      int seed = atoi(argv[2]);
      int raMethod = atof(argv[3]);
      double radius = atof(argv[4]); // Km
      int ulBW = atoi(argv[5]); // MHz
      int dlBW = atoi(argv[6]); // MHz
      int pdcchScheduler = atoi(argv[7]);
      int limitUEsToUlScheduler = atoi(argv[8]); // 0 - false ; 1 - true
      int maxMSGs4ToSchedule = atoi(argv[9]);
      int ulTDScheduler = atoi(argv[10]);
      int nUsersToFDScheduler = atoi(argv[11]);
      int ulFDScheduler = atoi(argv[12]);
      int semiSch = atoi(argv[13]); // only VoIP traffic -- 0 - false; 1 - true
      int nUE_HTC_VoIP = atoi(argv[14]);
      int nUE_HTC_Video = atoi(argv[15]);
      int nUE_HTC_CBR = atoi(argv[16]);
      int nUE_MTC_TimeDriven = atoi(argv[17]);
      int nUE_MTC_Emergency = atoi(argv[18]);
      int nUE_MTC_Video = atoi(argv[19]);
      Scenario_06(seed, raMethod, radius, ulBW, dlBW, pdcchScheduler, limitUEsToUlScheduler, maxMSGs4ToSchedule, ulTDScheduler, nUsersToFDScheduler, ulFDScheduler, semiSch, nUE_HTC_VoIP, nUE_HTC_Video, nUE_HTC_CBR, nUE_MTC_TimeDriven, nUE_MTC_Emergency, nUE_MTC_Video);
    }

    if (strcmp(argv[1], "scenario-07") == 0) {
      int seed = atoi(argv[2]);
      int raMethod = atof(argv[3]);
      double radius = atof(argv[4]); // Km
      int ulBW = atoi(argv[5]); // MHz
      int dlBW = atoi(argv[6]); // MHz
      int pdcchScheduler = atoi(argv[7]);
      int maxMSGs4ToSchedule = atoi(argv[8]);
      int ulTDScheduler = atoi(argv[9]);
      int nUsersToFDScheduler = atoi(argv[10]);
      int ulFDScheduler = atoi(argv[11]);
      int semiSch = atoi(argv[12]); // only VoIP traffic -- 0 - false; 1 - true
      int nUE_HTC_VoIP = atoi(argv[13]);
      int nUE_HTC_Video_GBR = atoi(argv[14]);
      int nUE_HTC_Video_nGBR = atoi(argv[15]);
      int nUE_HTC_CBR = atoi(argv[16]);
      int nUE_HTC_Game = atoi(argv[17]);
      int nUE_MTC_TimeDriven = atoi(argv[18]);
      int nUE_MTC_Emergency = atoi(argv[19]);
      Scenario_07(seed, raMethod, radius, ulBW, dlBW, pdcchScheduler, maxMSGs4ToSchedule, ulTDScheduler, nUsersToFDScheduler, ulFDScheduler, semiSch, nUE_HTC_VoIP, nUE_HTC_Video_GBR, nUE_HTC_Video_nGBR, nUE_HTC_CBR, nUE_HTC_Game, nUE_MTC_TimeDriven, nUE_MTC_Emergency);
    }

    if (strcmp(argv[1], "scenario-08") == 0) {
      int seed = atoi(argv[2]);
      int raMethod = atof(argv[3]);
      double radius = atof(argv[4]); // Km
      int ulBW = atoi(argv[5]); // MHz
      int dlBW = atoi(argv[6]); // MHz
      int nbUE_MTC = atoi(argv[7]);
      int acb_Dynamic = atoi(argv[8]);
      int eab_Dynamic = atoi(argv[9]);
      Scenario_08(seed, raMethod, radius, ulBW, dlBW, nbUE_MTC, acb_Dynamic, eab_Dynamic);
    }

    if (strcmp(argv[1], "scenario-09") == 0) {
      int seed = atoi(argv[2]);
      double radius = atof(argv[3]); // Km
      int ulBW = atoi(argv[4]); // MHz
      int dlBW = atoi(argv[5]); // MHz
      Scenario_09(seed, radius, ulBW, dlBW);
    }

    if (strcmp(argv[1], "scenario-10") == 0) {
      int seed = atoi(argv[2]);
      int raMethod = atof(argv[3]);
      double radius = atof(argv[4]); // Km
      int ulBW = atoi(argv[5]); // MHz
      int dlBW = atoi(argv[6]); // MHz
      int ulTDScheduler = atoi(argv[7]);
      int ulFDScheduler = atoi(argv[8]);
      int nUE_HTC_CBR = atoi(argv[9]);
      int nPRBs = atoi(argv[10]);
      int distance = atoi(argv[11]);
      Scenario_10(seed, raMethod, radius, ulBW, dlBW, ulTDScheduler, ulFDScheduler, nUE_HTC_CBR, nPRBs, distance);
    }

    if (strcmp(argv[1], "scenario-11") == 0) {
      if (strcmp(argv[2], "-h") == 0) {
        std::cout << "1 - seed" << std::endl;
      } else {
        int seed = atoi(argv[2]);
        int raMethod = atof(argv[3]);
        double radius = atof(argv[4]); // Km
        int ulBW = atoi(argv[5]); // MHz
        int dlBW = atoi(argv[6]); // MHz
        int pdcchScheduler = atoi(argv[7]);
        int maxMSGs4ToSchedule = atoi(argv[8]);
        int ulTDScheduler = atoi(argv[9]);
        int nUsersToFDScheduler = atoi(argv[10]);
        int ulFDScheduler = atoi(argv[11]);
        int semiSch = atoi(argv[12]); // only VoIP traffic -- 0 - false; 1 - true
        int nUE_HTC_VoIP = atoi(argv[13]);
        int nUE_HTC_Video_GBR = atoi(argv[14]);
        int nUE_HTC_Video_nGBR = atoi(argv[15]);
        int nUE_HTC_CBR = atoi(argv[16]);
        int nUE_MTC_TimeDriven = atoi(argv[17]);
        int nUE_MTC_Emergency = atoi(argv[18]);
        Scenario_11(seed, raMethod, radius, ulBW, dlBW, pdcchScheduler, maxMSGs4ToSchedule, ulTDScheduler, nUsersToFDScheduler, ulFDScheduler, semiSch, nUE_HTC_VoIP, nUE_HTC_Video_GBR, nUE_HTC_Video_nGBR, nUE_HTC_CBR, nUE_MTC_TimeDriven, nUE_MTC_Emergency);
      }
    }

    if (strcmp(argv[1], "scenario-12") == 0) {
      int seed = atoi(argv[2]);
      int raMethod = atof(argv[3]);
      double radius = atof(argv[4]); // Km
      int ulBW = atoi(argv[5]); // MHz
      int dlBW = atoi(argv[6]); // MHz
      int nbUE_HTC = atoi(argv[7]);
      int nbUE_MTC = atoi(argv[8]);
      //int useHandover = atoi(argv[9]);
      //int pdcchSch = atoi(argv[10]);
      // int effRA = atoi(argv[11]);
      // int scene = atoi(argv[12]);
      // int indoor = atoi(argv[13]);
      // std::cout << "THE ALGSTATUS = " << argv[11] << std::endl;

      Scenario_12(seed, raMethod, radius, ulBW, dlBW, nbUE_HTC, nbUE_MTC);
    }

    if (strcmp(argv[1], "scenario-13") == 0) {
      int seed = atoi(argv[2]);
      int raMethod = atof(argv[3]);
      double radius = atof(argv[4]); // Km
      int ulBW = atoi(argv[5]); // MHz
      int dlBW = atoi(argv[6]); // MHz
      int pdcchScheduler = atoi(argv[7]);
      int maxMSGs4ToSchedule = atoi(argv[8]);
      int dlTDScheduler = atoi(argv[9]);
      int nFlowsToFDScheduler = atoi(argv[10]);
      int dlFDScheduler = atoi(argv[11]);
      int semiSch = atoi(argv[12]); // only VoIP traffic -- 0 - false; 1 - true
      int nUE_HTC_VoIP = atoi(argv[13]);
      int nUE_HTC_Video_GBR = atoi(argv[14]);
      int nUE_HTC_Video_nGBR = atoi(argv[15]);
      int nUE_HTC_CBR = atoi(argv[16]);
      int nUE_HTC_Game = atoi(argv[17]);
      Scenario_13(seed, raMethod, radius, ulBW, dlBW, pdcchScheduler, maxMSGs4ToSchedule, dlTDScheduler, nFlowsToFDScheduler, dlFDScheduler, semiSch, nUE_HTC_VoIP, nUE_HTC_Video_GBR, nUE_HTC_Video_nGBR, nUE_HTC_CBR, nUE_HTC_Game);
    }

    if (strcmp(argv[1], "scenario-14") == 0) {
      int seed = atoi(argv[2]);
      double radius = atof(argv[3]); // Km
      int ulBW = atoi(argv[4]); // MHz
      int dlBW = atoi(argv[5]); // MHz
      int nUEs = atoi(argv[6]);
      int nAPPs = atoi(argv[7]);
      Scenario_14(seed, radius, ulBW, dlBW, nUEs, nAPPs);
    }

    if (strcmp(argv[1], "d2d-scenario") == 0) {
      if (argc == 2) {
        d2d_scenario();
      } else {
        std::cout << "    Incorrect parameters, please input the following values:" << std::endl;
        std::cout << "        <none>" << std::endl;
        std::cout << "    Example:" << std::endl << "        > ./LTE-Sim d2d-scenario" << std::endl;
      }
    }
  }
}
