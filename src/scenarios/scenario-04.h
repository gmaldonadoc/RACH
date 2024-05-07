/*
 * Author: Tiago Pedroso da Cruz de Andrade
 */

#include <cstring>
#include <cmath>

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

static void Scenario_04(int seed, int raMethod, double radius, int ulBW, int dlBW, int nbUE_HTC, int nbUE_MTC, int useHandover, int pdcchSch) {
  int idUniq = 0, iteratorDevice = 0, i = 0;
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
  informationManager->ra_PRACH_MaskIndex = 6;
  informationManager->numberOfRA_Preambles = 52;
  informationManager->preambleTransMax = 10;
  informationManager->ra_ResponseWindowSize = 5;
  informationManager->maxHarq_Msg3Tx = 5;
  informationManager->mac_ContentionResolutionTimer = 48;

  /* To Conventional Scheme */
  informationManager->backoffIndicatorGeneral = 2; // 20ms

  /* To ACB scheme */
  informationManager->acb_Dynamic = 0;
  informationManager->ac_BarringFactor = 0.9;
  informationManager->ac_BarringTime = 4;
  informationManager->acb_MonitoringPeriod = 75;

  /* To RRS scheme */
  informationManager->numberOfRA_Preambles_HTC = 22;

  /* To EAB scheme */
  informationManager->eab_Dynamic = 0;
  informationManager->eab_MonitoringPeriod = 500;

  /* To Specific Backoff scheme */
  informationManager->backoffIndicatorHTC = 2; // 20ms
  informationManager->backoffIndicatorMTC = 12; // 960ms

  /* To Cluster Group RA scheme */
  informationManager->numberOfPreamblesPerGroup = 15;
  informationManager->numberOfGroups = ceil(nbUE_MTC / (double) informationManager->numberOfPreamblesPerGroup);

  /* Aggregation level of PDCCH -> 0 - 1 CCE; 1 - 2 CCEs; 2 - 4 CCEs; 3 - 8 CCEs; -1 - Dynamic */
  informationManager->m_pdcch_format = 2;

  /* Create Channels */
  LteChannel *dlCh = new LteChannel();
  LteChannel *ulCh = new LteChannel();

  /* Create Spectrum */
  BandwidthManager *spectrum = new BandwidthManager(ulBW, dlBW, 0, 0);

  /* Create Cell */
  Cell *cell = networkManager->CreateCell(40000, radius, 0.100, 0, 0);

  /* Create ENodeB */
  ENodeB *enb = networkManager->CreateEnodeb(50000, cell, 0, 0, dlCh, ulCh, spectrum);

  /**/
  enb->SetP0NominalPusch(-54.0); // -126 to 24
  enb->SetAlphaPL(0.6); // 0, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9 and 1
  enb->SetPreambleInitialReceivedTargetPower(-110.0); // -120, -118, -116, -114, -112, -110, -108, -106, -104, -102, -100, -98, -96, -94, -92 and -90dBm
  enb->SetPowerRampingStep(2.0); // 0, 2, 4 and 6dB
  
  /**/
  enb->SetRrmEntity();
  enb->GetRrmEntity()->SetPdcchScheduler();
  enb->GetRrmEntity()->GetPdcchScheduler()->SetPDCCHModeType(pdcchSch);
  enb->GetRrmEntity()->GetPdcchScheduler()->SetMaxMSGs4ToSchedule(4);
  enb->GetRrmEntity()->GetPdcchScheduler()->SetLimitUEsToFD(true);
  enb->GetRrmEntity()->GetPdcchScheduler()->SetnUEsToDL(10);
  
  /**/
  enb->GetRrmEntity()->SetDLScheduler();
  enb->GetRrmEntity()->SetULScheduler();

  /* user equipments are distributed uniformly into a cell */
  vector<CartesianCoordinates*> *positions;

  if (useHandover == 1) {
    positions = GetUniformUsersDistributionCarlos(cell->GetIdCell(), nbUE_HTC + nbUE_MTC + (64 - informationManager->numberOfRA_Preambles));
  } else {
    positions = GetUniformUsersDistributionCarlos(cell->GetIdCell(), nbUE_HTC + nbUE_MTC);
  }

  if (nbUE_MTC > 0) {
    for (i = 0; i < nbUE_MTC; i++) {
      posX = positions->at(iteratorDevice)->GetCoordinateX();
      posY = positions->at(iteratorDevice)->GetCoordinateY();

      UserEquipment *ue = new UserEquipment(idUniq, posX, posY, 0, 0, cell, enb, false, Mobility::CONSTANT_POSITION);

      /* Set device type to MTC */
      ue->SetDeviceType(1);

      /* Configure DRX Manager */
      //ue->SetDRXManager();

      /* Configure Energy Consumption */
      ue->SetConsumptionModel("3GPP");

      /* Configure Battery Model */
      ue->SetBatteryModel();

      /* set the access class */
      ue->SetDeviceAccessClass(iteratorDevice % 10);

      ue->GetProtocolStack()->GetMacEntity()->SetQCI(9, 9, 300, false, 3);

      ue->GetPhy()->SetDlChannel(enb->GetPhy()->GetDlChannel());
      ue->GetPhy()->SetUlChannel(enb->GetPhy()->GetUlChannel());

      networkManager->GetUserEquipmentContainer()->push_back(ue);

      /* register UE to the eNodeB */
      enb->RegisterUserEquipment(ue);

      /* define the channel realization */
      MacroCellUrbanAreaChannelRealization *c_dl = new MacroCellUrbanAreaChannelRealization(enb, ue);
      c_dl->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
      enb->GetPhy()->GetDlChannel()->GetPropagationLossModel()->AddChannelRealization(c_dl);
      MacroCellUrbanAreaChannelRealization *c_ul = new MacroCellUrbanAreaChannelRealization(ue, enb);
      c_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
      enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(c_ul);
      
      idUniq++;
      iteratorDevice++;

      ue->StartRandomAccessProcedure(rndGenerator->Uniform(0.0015, 0.002));
    }
  }

  if (nbUE_HTC > 0) {
    for (i = 0; i < nbUE_HTC; i++) {
      posX = positions->at(iteratorDevice)->GetCoordinateX();
      posY = positions->at(iteratorDevice)->GetCoordinateY();

      UserEquipment *ue = new UserEquipment(idUniq, posX, posY, 0, 0, cell, enb, false, Mobility::CONSTANT_POSITION);

      /* set the equipment as HTC */
      ue->SetDeviceType(0);

      /* Configure DRX Manager */
      //ue->SetDRXManager();

      /* Configure Energy Consumption */
      ue->SetConsumptionModel("3GPP");

      /* Configure Battery Model */
      ue->SetBatteryModel();

      /* set the access class */
      ue->SetDeviceAccessClass(iteratorDevice % 10);

      ue->GetProtocolStack()->GetMacEntity()->SetQCI(9, 9, 300, false, 3);

      ue->GetPhy()->SetDlChannel(enb->GetPhy()->GetDlChannel());
      ue->GetPhy()->SetUlChannel(enb->GetPhy()->GetUlChannel());

      networkManager->GetUserEquipmentContainer()->push_back(ue);

      /* register UE to the eNodeB */
      enb->RegisterUserEquipment(ue);

      /* define the channel realization */
      MacroCellUrbanAreaChannelRealization *c_dl = new MacroCellUrbanAreaChannelRealization(enb, ue);
      c_dl->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
      enb->GetPhy()->GetDlChannel()->GetPropagationLossModel()->AddChannelRealization(c_dl);
      MacroCellUrbanAreaChannelRealization *c_ul = new MacroCellUrbanAreaChannelRealization(ue, enb);
      c_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
      enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(c_ul);
      
      idUniq++;
      iteratorDevice++;

      ue->StartRandomAccessProcedure(rndGenerator->Uniform(0.0015, 0.002));
    }
  }

  if (useHandover == 1) {
    for (i = 0; i < (64 - informationManager->numberOfRA_Preambles); i++) {
      posX = positions->at(iteratorDevice)->GetCoordinateX();
      posY = positions->at(iteratorDevice)->GetCoordinateY();

      UserEquipment *ue = new UserEquipment(idUniq, posX, posY, 0, 0, cell, enb, false, Mobility::CONSTANT_POSITION);

      /* set the equipment as HTC */
      ue->SetDeviceType(0);

      /* Configure DRX Manager */
      //ue->SetDRXManager();

      /* Configure Energy Consumption */
      ue->SetConsumptionModel("3GPP");

      /* Configure Battery Model */
      ue->SetBatteryModel();

      /* set the access class */
      ue->SetDeviceAccessClass(iteratorDevice % 10);

      ue->GetProtocolStack()->GetMacEntity()->SetQCI(9, 9, 300, false, 3);

      ((UeMacEntity *) ue->GetProtocolStack()->GetMacEntity())->SetPreambleSequence(63 - i);

      ue->GetPhy()->SetDlChannel(enb->GetPhy()->GetDlChannel());
      ue->GetPhy()->SetUlChannel(enb->GetPhy()->GetUlChannel());

      networkManager->GetUserEquipmentContainer()->push_back(ue);

      /* register UE to the eNodeB */
      enb->RegisterUserEquipment(ue);

      /* define the channel realization */
      MacroCellUrbanAreaChannelRealization *c_dl = new MacroCellUrbanAreaChannelRealization(enb, ue);
      c_dl->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
      enb->GetPhy()->GetDlChannel()->GetPropagationLossModel()->AddChannelRealization(c_dl);
      MacroCellUrbanAreaChannelRealization *c_ul = new MacroCellUrbanAreaChannelRealization(ue, enb);
      c_ul->SetChannelType(ChannelRealization::CHANNEL_TYPE_PED_A);
      enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(c_ul);

      idUniq++;
      iteratorDevice++;

      ue->StartRandomAccessProcedure(rndGenerator->Uniform(0.0015, 0.002));
    }
  }

  positions->clear();
  delete positions;

  /*
   * Initialize DRX management cycle of all UEs.
   * Important: It must go after the declaration of all UEs and before running the simulation
   */
  enb->InitDRXCycle();

  /*
   * Initialize Energy Consumption management cycle of all UEs.
   * Important: It must go after the declaration of all UEs and before running the simulation and after DRX initiation
   */
  enb->InitConsumptionEnergyCycle();

  /*
   * Initialize Frame Structure cycle of eNB.
   * Important: It must go after the declaration of DRX initialization and before Running the simulation
   */
  frameManager->Start();

  simulator->Run();

  /**/
  UserEquipment* uepointer;
  vector<UserEquipment*>::iterator iter;
  double consumoTotal = 0.0, consumoue = 0.0;
  int timerRx = 0, timerRxTx = 0, timerTx = 0, timerNoData = 0, timerLightSleep = 0, timerDeepSleep = 0, timerLightActive = 0, idue = 0;
  int counter = 0;
  //std::cerr << Simulator::Init()->Now() << std::endl;

  std::cerr << "--------------------------" << std::endl;
  for (iter = networkManager->GetUserEquipmentContainer()->begin(); iter != networkManager->GetUserEquipmentContainer()->end(); iter++) {
    counter++;
    uepointer = *iter;
    consumoue = uepointer->GetConsumptionModel()->getConsumption();
    consumoTotal += consumoue;
    timerRx = uepointer->GetConsumptionModel()->gettActiveRx();
    timerRxTx = uepointer->GetConsumptionModel()->gettActiveRxTx();
    timerTx = uepointer->GetConsumptionModel()->gettActiveTx();
    timerNoData = uepointer->GetConsumptionModel()->gettActiveNoData();
    timerLightSleep = uepointer->GetConsumptionModel()->gettLightSleep();
    timerDeepSleep = uepointer->GetConsumptionModel()->gettDeepSleep();
    timerLightActive = uepointer->GetConsumptionModel()->gettLightActive();
    idue = uepointer->GetIDNetworkNode();
    std::cerr << "UE #" << idue
            << " --- Energia Consumida [J]: " << consumoue
            << " - Tempo Tx [ms]: " << timerTx
            << " - Tempo Rx [ms]: " << timerRx
            << " - Tempo RxTx [ms]: " << timerRxTx
            << " - Tempo NoData [ms]: " << timerNoData
            << " - Tempo LightSleep [ms]: " << timerLightSleep
            << " - Tempo DeepSleep [ms]: " << timerDeepSleep
            << " - Tempo Wakeup [ms]: " << uepointer->GetConsumptionModel()->gettSleepToActive()
            << " - Tempo Powerdown [ms]: " << uepointer->GetConsumptionModel()->gettActiveToSleep()
            << " - Bateria: " << uepointer->GetBatteryModel()->GetCurrentCapacity()
            << "mA " << uepointer->GetBatteryModel()->GetBatteryCharge() << "%"
            << std::endl;
  }
  std::cerr << "--------------------------" << std::endl;

  std::cerr << "Energia Consumida total [J]: " << consumoTotal << std::endl;
  /**/
}