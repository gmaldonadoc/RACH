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
#include "../energyConsumption/ue-energy-model.h"

static void Scenario_08(int seed, int raMethod, double radius, int ulBW, int dlBW, int nbUE_MTC, int acb_Dynamic, int eab_Dynamic) {
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
  informationManager->numberOfRA_Preambles = 52;
  informationManager->preambleTransMax = 10;
  informationManager->ra_PRACH_MaskIndex = 6;

  /* To Conventional Scheme */
  informationManager->backoffIndicatorGeneral = 2; // 20ms

  /* To ACB scheme */
  informationManager->acb_Dynamic = acb_Dynamic;
  if (acb_Dynamic == 1) {
    informationManager->ac_BarringFactor = 1;
  } else {
    informationManager->ac_BarringFactor = 0.9;
  }
  informationManager->ac_BarringTime = 4;
  informationManager->acb_MonitoringPeriod = 75;

  /* To RRS scheme */
  informationManager->numberOfRA_Preambles_HTC = 22;

  /* To EAB scheme */
  informationManager->eab_Dynamic = eab_Dynamic;
  informationManager->eab_MonitoringPeriod = 500;

  /* To Specific Backoff scheme */
  informationManager->backoffIndicatorHTC = 2; // 20ms
  informationManager->backoffIndicatorMTC = 12; // 960ms

  /* Aggregation level of PDCCH -> 4 CCEs per DCI */
  informationManager->m_pdcch_format = 2;

  /* Create Channels */
  LteChannel *dlCh = new LteChannel();
  LteChannel *ulCh = new LteChannel();

  /* Create Spectrum */
  BandwidthManager *spectrum = new BandwidthManager(ulBW, dlBW, 0, 0);

  /* Create Cell */
  Cell *cell = networkManager->CreateCell(1, radius, 0.100, 0, 0);

  /* Create ENodeB */
  ENodeB *enb = networkManager->CreateEnodeb(2, cell, 0, 0, dlCh, ulCh, spectrum);

  /**/
  enb->SetPreambleInitialReceivedTargetPower(-110.0);
  enb->SetPowerRampingStep(4.0);

  /**/
  enb->SetP0NominalPusch(-103.0);
  enb->SetAlphaPL(1.0);

  /* Set the parameters of the PDCCH Manager*/
  enb->SetPdcchScheduler();
  enb->GetPdcchScheduler()->SetPDCCHModeType(1);
  enb->GetPdcchScheduler()->SetMaxMSGs4ToSchedule(4);
  enb->GetPdcchScheduler()->SetLimitUEsToFD(true);

  /**/
  enb->SetDLScheduler(ENodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR);

  /**/
  enb->SetULScheduler(ENodeB::ULScheduler_TYPE_FME);

  /**/
  ((UplinkPacketScheduler *) enb->GetULScheduler())->SetUplinkTDScheduler(ENodeB::ULScheduler_TYPE_FME);

  /**/
  ((UplinkPacketScheduler *) enb->GetULScheduler())->SetUplinkFDScheduler(ENodeB::ULScheduler_TYPE_FME);

  /* user equipments are distributed uniformly into a cell */
  vector<CartesianCoordinates*> *positions;
  positions = GetUniformUsersDistributionCarlos(cell->GetIdCell(), nbUE_MTC);

  if (nbUE_MTC > 0) {
    string _file(path + "src/arrival/probability-6-10.txt");
    vector<TimeArrival*> *arrivals = betaArrival->buildArrival(6, 10, nbUE_MTC, _file);

    for (iteratorSlot = 0; iteratorSlot < arrivals->size(); iteratorSlot++) {

      for (i = 0; i < arrivals->at(iteratorSlot)->getNumberArrivals(); i++) {
        posX = positions->at(iteratorDevice)->GetCoordinateX();
        posY = positions->at(iteratorDevice)->GetCoordinateY();

        UserEquipment *ue = new UserEquipment(idUniq, posX, posY, 0, 0, cell, enb, false, Mobility::CONSTANT_POSITION);

        /* set the equipment as MTC */
        ue->SetDeviceType(1);

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
        enb->GetPhy()->GetDlChannel()->GetPropagationLossModel()->AddChannelRealization(c_dl);
        MacroCellUrbanAreaChannelRealization *c_ul = new MacroCellUrbanAreaChannelRealization(ue, enb);
        enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(c_ul);

        idUniq++;
        iteratorDevice++;

        ue->StartRandomAccessProcedure(arrivals->at(iteratorSlot)->getTimeArrivals());
      }
    }

    arrivals->clear();
    delete arrivals;
  }

  positions->clear();
  delete positions;

  /*
   * Initialize Frame Structure cycle of eNB.
   * Important: It must go after the declaration of DRX initialization and before Running the simulation
   */
  frameManager->Start();

  simulator->Run();
}