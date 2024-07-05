/*
 * Author: Tiago Pedroso da Cruz de Andrade
 */

#include <cstring>
#include <cmath>
#include <assert.h>
#include <cstring>
#include <stdlib.h>
#include <cmath>
#include <iomanip>

#include "../componentManagers/InformationManager.h"
#include "../channel/LteChannel.h"
#include "../core/spectrum/bandwidth-manager.h"
#include "../networkTopology/Cell.h"
#include "../core/eventScheduler/simulator.h"
#include "../flows/application/InfiniteBuffer.h"
#include "../flows/QoS/QoSParameters.h"
#include "../componentManagers/FrameManager.h"
#include "../componentManagers/FlowsManager.h"
#include "../device/ENodeB.h"
#include "../utility/UsersDistribution.h"
#include "../flows/application/WEB.h"
#include "../arrival/BetaArrival.h"
#include "../energyConsumption/ue-consumption-model.h"
#include "../drx/drx-manager.h"
#include "../protocolStack/mac/rrm/rrm-entity.h"
#include "../channel/propagation-model/macrocell-rural-area-channel-realization.h"

static void Scenario_12(int seed, int raMethod, double radius, int ulBW, int dlBW,
                        int nbUE_HTC, int nbUE_MTC)
{

  int idUniq = 3, iteratorSlot = 0, iteratorDevice = 0, i = 0;
  double posX = 0, posY = 0;

  /* Create COMPONENT MANAGERS */
  Simulator *simulator = Simulator::Init();
  FrameManager *frameManager = FrameManager::Init();
  NetworkManager *networkManager = NetworkManager::Init();
  FlowsManager *flowsManager = FlowsManager::Init();
  InformationManager *informationManager = InformationManager::Init();
  Random *rndGenerator = Random::Init(seed);
  BetaArrival *betaArrival = BetaArrival::Init();

  /* Default PRACH Configuration */
  informationManager->ra_Method = raMethod;
  informationManager->maxHarq_Msg3Tx = 5;
  informationManager->mac_ContentionResolutionTimer = 48;
  informationManager->numberOfRA_Preambles = 54;
  informationManager->preambleTransMax = 10;
  informationManager->ra_PRACH_MaskIndex = 6;

  /* To Conventional Scheme */
  informationManager->backoffIndicatorGeneral = 2; // 20ms

  /* To ACB scheme */
  informationManager->acb_Dynamic = false;
  informationManager->ac_BarringFactor = 0.9;
  informationManager->ac_BarringTime = 4;
  informationManager->acb_MonitoringPeriod = 75;

  /* To RRS scheme */
  informationManager->numberOfRA_Preambles_HTC = 22;

  /* To EAB scheme */
  informationManager->eab_Dynamic = 0;
  informationManager->eab_MonitoringPeriod = 500;

  /* To Specific Backoff scheme */
  informationManager->backoffIndicatorHTC = 2;  // 20ms
  informationManager->backoffIndicatorMTC = 12; // 960ms

  /* Aggregation level of PDCCH -> 4 CCEs per DCI */
  informationManager->m_pdcch_format = 2;

  /* Create Channels */
  LteChannel *dlCh = new LteChannel();
  LteChannel *ulCh = new LteChannel();

  /* Create Spectrum */
  BandwidthManager *spectrum = new BandwidthManager(ulBW, dlBW, 0, 0);

  /* Create Cell */
  Cell *cell = networkManager->CreateCell(1, radius, 0.060, 0, 0);

  /* Create ENodeB */
  ENodeB *enb = networkManager->CreateEnodeb(2, cell, 0, 0, dlCh, ulCh, spectrum);

  /**/
  // -120, -118, -116, -114, -112, -110, -108, -106, -104, -102, -100, -98, -96, -94, -92 -90
  enb->SetPreambleInitialReceivedTargetPower(-118.0);
  enb->SetPowerRampingStep(2.0); // 0, 2, 4 and 6dB

  /**/
  enb->SetP0NominalPusch(-110.0); // -126 to 24dbm
  enb->SetAlphaPL(1.0);           // 0, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9 and 1

  /**/
  enb->SetRrmEntity();
  enb->GetRrmEntity()->SetPdcchScheduler();
  enb->GetRrmEntity()->GetPdcchScheduler()->SetPDCCHModeType(7);
  enb->GetRrmEntity()->GetPdcchScheduler()->SetMaxMSGs4ToSchedule(4);
  enb->GetRrmEntity()->GetPdcchScheduler()->SetLimitUEsToFD(true);
  enb->GetRrmEntity()->GetPdcchScheduler()->SetnUEsToDL(10);

  /**/
  enb->GetRrmEntity()->SetDLScheduler();
  enb->GetRrmEntity()->SetULScheduler();

  /* user equipments are distributed uniformly into a cell */
  vector<CartesianCoordinates *> *positions;
  // positions = GetUniformUsersDistributionCarlos(cell->GetIdCell(), nbUE_HTC + nbUE_MTC);
  positions = GetUniformUsersDistribution(cell->GetIdCell(), nbUE_HTC + nbUE_MTC);

  if (nbUE_MTC > 0)
  {
    string _file(path + "src/arrival/probability-6-10.txt");
    vector<TimeArrival *> *arrivals = betaArrival->buildArrival(6, 10, nbUE_MTC, _file);

    for (iteratorSlot = 0; iteratorSlot < arrivals->size(); iteratorSlot++)
    {

      for (i = 0; i < arrivals->at(iteratorSlot)->getNumberArrivals(); i++)
      {
        posX = positions->at(iteratorDevice)->GetCoordinateX();
        posY = positions->at(iteratorDevice)->GetCoordinateY();

        UserEquipment *ue = new UserEquipment(idUniq, posX, posY, 0, 0, cell, enb, false, Mobility::CONSTANT_POSITION);

        /* set the equipment as MTC */
        ue->SetDeviceType(1);

        /* Configure Energy Consumption */
        ue->SetConsumptionModel("LAURIDSEN");

        /* Configure Battery Model */
        ue->SetBatteryModel();

        /* set the access class */
        ue->SetDeviceAccessClass(iteratorDevice % 10);

        ue->GetProtocolStack()->GetMacEntity()->SetQCI(9, 9, 300, false, 3);

        ue->GetPhy()->SetDlChannel(enb->GetPhy()->GetDlChannel());
        ue->GetPhy()->SetUlChannel(enb->GetPhy()->GetUlChannel());

        ((UeLtePhy *)ue->GetPhy())->m_use_efficient_ra = false;

        networkManager->GetUserEquipmentContainer()->push_back(ue);

        /* register UE to the eNodeB */
        enb->RegisterUserEquipment(ue);

        /* define the channel realization -- DL */
        MacroCellUrbanAreaChannelRealization *c_dl = new MacroCellUrbanAreaChannelRealization(enb, ue);
        c_dl->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
        enb->GetPhy()->GetDlChannel()->GetPropagationLossModel()->AddChannelRealization(c_dl);

        /* define the channel realization -- UL */
        MacroCellUrbanAreaChannelRealization *c_ul = new MacroCellUrbanAreaChannelRealization(ue, enb);
        c_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
        enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(c_ul);

        idUniq++;
        iteratorDevice++;

        ue->StartRandomAccessProcedure(arrivals->at(iteratorSlot)->getTimeArrivals());
      }
    }

    arrivals->clear();
    delete arrivals;
  }

  /* Initial Next RA Uniform variable */
  int HTCCounter = 0;
  double nextHTC[nbUE_HTC]; // Create a vector for initial RA

  while (HTCCounter < nbUE_HTC)
  {
    nextHTC[HTCCounter] = 0.005 + (round(rndGenerator->Uniform(0.0, 1.0) * 1000)) / 1000;
    HTCCounter++;
  }

  for (i = 0; i < nbUE_HTC; i++)
  {
    posX = positions->at(iteratorDevice)->GetCoordinateX();
    posY = positions->at(iteratorDevice)->GetCoordinateY();

    UserEquipment *ue = new UserEquipment(idUniq, posX, posY, 0, 0, cell, enb, false, Mobility::CONSTANT_POSITION);

    /* set the equipment as HTC */
    ue->SetDeviceType(0);

    /* Configure Energy Consumption */
    ue->SetConsumptionModel("LAURIDSEN");

    /* Configure Battery Model */
    ue->SetBatteryModel();

    ue->SetIndoorFlag(false);

    /* set the access class */
    ue->SetDeviceAccessClass(iteratorDevice % 10);

    ue->GetProtocolStack()->GetMacEntity()->SetQCI(9, 9, 300, false, 3);

    ue->GetPhy()->SetDlChannel(enb->GetPhy()->GetDlChannel());
    ue->GetPhy()->SetUlChannel(enb->GetPhy()->GetUlChannel());

    ((UeLtePhy *)ue->GetPhy())->m_use_efficient_ra = false;

    networkManager->GetUserEquipmentContainer()->push_back(ue);

    /* register UE to the eNodeB */
    enb->RegisterUserEquipment(ue);

    /* define the channel realization */

    ChannelRealization *c_dl;
    ChannelRealization *c_ul;

    c_dl = new MacroCellUrbanAreaChannelRealization(enb, ue);
    c_ul = new MacroCellUrbanAreaChannelRealization(ue, enb);

    c_dl->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
    enb->GetPhy()->GetDlChannel()->GetPropagationLossModel()->AddChannelRealization(c_dl);

    c_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
    enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(c_ul);

    /* */
    int maxRndNumbers = 20, rndIterator = 0, k;
    double rndNumbers[20], r, m_stateDuration, m_endOn;
    double nextRA = nextHTC[i];

    c_dl->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
    enb->GetPhy()->GetDlChannel()->GetPropagationLossModel()->AddChannelRealization(c_dl);

    c_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
    enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(c_ul);

    for (k = 0; k < maxRndNumbers; k++)
    {
      rndNumbers[k] = rndGenerator->Uniform(0., 1.);
    }

    while (nextRA < 10)
    {
      ue->StartRandomAccessProcedure(nextRA);

      r = rndNumbers[rndIterator % maxRndNumbers];
      rndIterator++;

      m_endOn = -3 * log(1 - r);

      r = rndNumbers[rndIterator % maxRndNumbers];
      rndIterator++;

      m_stateDuration = -2.23 * log(1 - r);
      if (m_stateDuration > 6.9)
      {
        m_stateDuration = 6.9;
      }

      nextRA += round((m_endOn + (double)(floor(m_stateDuration * 1000) / 1000)) * 1000) / 1000;
      nextRA = 10;
    }

    idUniq++;
    iteratorDevice++;
  }

  positions->clear();
  delete positions;
  enb->InitConsumptionEnergyCycle();
  frameManager->Start();
  simulator->SetStop(12.5);
  simulator->Run();

  /**/
  UserEquipment *uepointer;
  vector<UserEquipment *>::iterator iter;
  double consumptionTotal = 0.0, consumptionue = 0.0, consumptionueConn = 0.0;
  double totalConnCons = 0.0;
  int timerRx = 0, timerRxTx = 0, timerTx = 0, timerConn = 0, timerTotal = 0;
  int totaltimerRx = 0, totaltimerRxTx = 0, totaltimerTx = 0, totaltimerNoData = 0;
  double consumptionTx = 0.0, consumptionRx = 0.0;
  double consumptionRxTx = 0.0, consumptionNoData = 0.0;
  double totalconsumptionTx = 0.0, totalconsumptionRx = 0.0;
  double totalconsumptionRxTx = 0.0, totalconsumptionNoData = 0.0;
  int timerNoData = 0, timerLightSleep = 0, timerDeepSleep = 0, timerLightActive = 0, idue = 0;
  int totalConn = 0, totalRxTx = 0;
  int totalbitsUL = 0, totalbitsDL = 0;
  int uebitsUL = 0, uebitsDL = 0;
  int counter = 0;
  int totaltimerVideo = 0, totaltimerVoip = 0, totaltimerCBR = 0;
  double totalconsumptionVideo = 0.0, totalconsumptionVoip = 0.0, totalconsumptionCBR = 0.0;
  // std::cerr << Simulator::Init()->Now() << std::endl;

  std::cerr << "--------------------------" << std::endl;
  for (iter = networkManager->GetUserEquipmentContainer()->begin();
       iter != networkManager->GetUserEquipmentContainer()->end(); iter++)
  {

    uepointer = *iter;
    consumptionue = uepointer->GetConsumptionModel()->getConsumption();
    consumptionueConn = uepointer->GetConsumptionModel()->getConnectedConsumption();
    consumptionTotal += consumptionue;

    timerRx = uepointer->GetConsumptionModel()->gettActiveRx();
    timerTx = uepointer->GetConsumptionModel()->gettActiveTx();
    timerRxTx = uepointer->GetConsumptionModel()->gettActiveRxTx();
    timerNoData = uepointer->GetConsumptionModel()->gettActiveNoData();
    timerTotal = timerTx + timerRx + timerRxTx + timerNoData;

    totaltimerRx += timerRx;
    totaltimerTx += timerTx;
    totaltimerRxTx += timerRxTx;
    totaltimerNoData += timerNoData;
    totalConn += timerRx + timerTx + timerRxTx;

    consumptionRx = uepointer->GetConsumptionModel()->getConsumptionInRx();
    consumptionTx = uepointer->GetConsumptionModel()->getConsumptionInTx();
    consumptionRxTx = uepointer->GetConsumptionModel()->getConsumptionInRxTx();
    consumptionNoData = uepointer->GetConsumptionModel()->getConsumptionInNoData();

    totalconsumptionRx += consumptionRx;
    totalconsumptionTx += consumptionTx;
    totalconsumptionRxTx += consumptionRxTx;
    totalconsumptionNoData += consumptionNoData;
    totalConnCons += consumptionueConn;

    idue = uepointer->GetIDNetworkNode();

    /* if((idue - 3) < nUE_video){ // Video */
    /*   totaltimerVideo += timerTx; */
    /*   totalconsumptionVideo += consumptionue; */
    /* } else if((idue - 3) < nUE_video + nUE_voip){ // VoiP */
    /*   totaltimerVoip += timerTx; */
    /*   totalconsumptionVoip += consumptionue; */
    /* } else { // CBR */
    /*   totaltimerCBR += timerTx; */
    /*   totalconsumptionCBR += consumptionue; */
    /* } */

    std::cout << std::setprecision(4) << std::fixed;
    std::cout << "UE  #" << std::setw(3) << idue << " | "
              << " Tx: " << std::setw(8) << timerTx << " | "
              << " Rx: " << std::setw(8) << timerRx << " | "
              << " RxTx: " << std::setw(8) << timerRxTx << " | "
              << "  ND : " << std::setw(8) << timerNoData << " | "
              << "  ALL: " << std::setw(8) << timerTotal << " | "
              << "ETx: " << std::setw(8) << consumptionTx << " | "
              << "ERx: " << std::setw(8) << consumptionRx << " | "
              << "ERxTx: " << std::setw(8) << consumptionRxTx << " | "
              << "E ND : " << std::setw(8) << consumptionNoData << " | "
              << "E ALL: " << std::setw(8) << consumptionue
              << std::endl;
  }
  std::cout << "END--------------------------" << std::endl;
  std::cout << "Total Consumption [J]: " << consumptionTotal << std::endl;
  std::cout << "Total Conn  [ms]: " << totalConn << std::endl;
  std::cout << "Total Rx    [ms]: " << totaltimerRx << std::endl;
  std::cout << "Total Tx    [ms]: " << totaltimerTx << std::endl;
  std::cout << "Total RxTx  [ms]: " << totaltimerRxTx << std::endl;
  std::cout << "Total ND    [ms]: " << totaltimerNoData << std::endl;

  std::cout << "ETotal Conn  [J]: " << totalConnCons << std::endl;
  std::cout << "ETotal Rx    [J]: " << totalconsumptionRx << std::endl;
  std::cout << "ETotal Tx    [J]: " << totalconsumptionTx << std::endl;
  std::cout << "ETotal RxTx  [J]: " << totalconsumptionRxTx << std::endl;
  std::cout << "ETotal ND    [J]: " << totalconsumptionNoData << std::endl;

  std::cout << "ETotal Video [J]: " << totalconsumptionVideo << std::endl;
  std::cout << "ETotal Voip  [J]: " << totalconsumptionVoip << std::endl;
  std::cout << "ETotal CBR   [J]: " << totalconsumptionCBR << std::endl;

  counter++;
  /**/

  // for (auto &&i : *(networkManager->GetENodeBContainer()))
  // {

  //   auto m_RAPreambleContainer = dynamic_cast<EnbMacEntity *>(i->GetProtocolStack()->GetM_MAC())->GetAllPreambles();
  //   for (auto &&j : m_RAPreambleContainer)
  //   {
  //     std::cout << "Preambulos: " << j << std::endl;
  //   }
 
  // }

  // for (auto &&i : *(networkManager->GetUserEquipmentContainer()))
  // {
  //   auto all_users_position = i->GetAllUsersCoordinates();
  //   for (auto &&j : all_users_position)
  //   {   
  //       double timeInicioRA = nextRA; // Extraer el tiempo de inicio del procedimiento de acceso aleatorio
  //     std::cout << "UserID: " << i->GetIDNetworkNode() << "\t"
  //               << "PositionX: " << j.GetCoordinateX() << "\t"
  //               << "PositionY:" << j.GetCoordinateY() << "\t"
  //               << "Time: " << Simulator::Init()->Now() << "\t" // Imprimir el tiempo actual del simulador
  //               << "TimeInicioRA: " << timeInicioRA << std::endl; // Imprimir el tiempo de inicio del procedimiento de acceso aleatorio
  //   }
  // }
}
