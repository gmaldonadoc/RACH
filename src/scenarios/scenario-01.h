/*
 * Author: Tiago Pedroso da Cruz de Andrade
 */

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

static void Scenario_01(int raMethod, int seed, int nbUE_HTC, int nbUE_MTC, int RACHIndex, int preambleTransMax, int backoffIndex_general, int backoffIndex_HTC, int backoffIndex_MTC, int nbTotalPreambles, int nbHTCPreambles, double acbBF, double acbBT) {
  int idUniq = 0;
  
  // Create COMPONENT MANAGERS
  Simulator *simulator = Simulator::Init();
  FrameManager *frameManager = FrameManager::Init();
  NetworkManager *networkManager = NetworkManager::Init();
  FlowsManager *flowsManager = FlowsManager::Init();
  InformationManager *informationManager = InformationManager::Init();
  Random *rndGenerator = Random::Init(seed);

  informationManager->ra_Method = raMethod;
  informationManager->numberOfRA_Preambles = nbTotalPreambles;
  informationManager->preambleTransMax = preambleTransMax;
  informationManager->ra_PRACH_MaskIndex = RACHIndex;
  
  informationManager->backoffIndicatorGeneral = backoffIndex_general;
  informationManager->backoffIndicatorHTC = backoffIndex_HTC;
  informationManager->backoffIndicatorMTC = backoffIndex_MTC;
  
  informationManager->ac_BarringFactor = acbBF;
  informationManager->ac_BarringTime = acbBT;
  
  informationManager->numberOfRA_Preambles_HTC = nbHTCPreambles;

  // Create CHANNELS
  LteChannel *dlCh = new LteChannel();
  LteChannel *ulCh = new LteChannel();

  // Create SPECTRUM
  BandwidthManager *bandwidth = new BandwidthManager(5, 5, 0, 0);

  // Create CELL
  Cell *cell = networkManager->CreateCell(idUniq++, 0.5, 0.050, 0, 0);

  // Create ENodeB
  ENodeB *enb = networkManager->CreateEnodeb(idUniq++, cell, 0, 0, dlCh, ulCh, bandwidth);

  //user equipments are distributed uniformly into a cell
  vector<CartesianCoordinates*> *positions = GetUniformUsersDistribution(cell->GetIdCell(), nbUE_HTC + nbUE_MTC);

  double posX = 0;
  double posY = 0;

  //Create UEs HTC
  for (int i = 0; i < nbUE_HTC; i++) {
    posX = positions->at(i)->GetCoordinateX();
    posY = positions->at(i)->GetCoordinateY();
    
    UserEquipment *ue = new UserEquipment(idUniq++, posX, posY, 0, 0, cell, enb, false, Mobility::CONSTANT_POSITION);
    
    // set device type to HTC
    ue->SetDeviceType(0);
    
    // set the access class
    ue->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));

    ue->GetPhy()->SetDlChannel(enb->GetPhy()->GetDlChannel());
    ue->GetPhy()->SetUlChannel(enb->GetPhy()->GetUlChannel());

    networkManager->GetUserEquipmentContainer()->push_back(ue);

    // register UE to the eNodeB
    enb->RegisterUserEquipment(ue);

    // define the channel realization
    MacroCellUrbanAreaChannelRealization *c_dl = new MacroCellUrbanAreaChannelRealization(enb, ue);
    enb->GetPhy()->GetDlChannel()->GetPropagationLossModel()->AddChannelRealization(c_dl);
    MacroCellUrbanAreaChannelRealization *c_ul = new MacroCellUrbanAreaChannelRealization(ue, enb);
    enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(c_ul);

    ue->StartRandomAccessProcedure(0.000);
  }
  
  //Create UEs MTC
  for (int i = 0; i < nbUE_MTC; i++) {
    posX = positions->at(nbUE_HTC + i)->GetCoordinateX();
    posY = positions->at(nbUE_HTC + i)->GetCoordinateY();

    UserEquipment *ue = new UserEquipment(idUniq++, posX, posY, 0, 0, cell, enb, false, Mobility::CONSTANT_POSITION);
    
    // set device type to MTC
    ue->SetDeviceType(1);
    
    // set the access class
    ue->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));

    ue->GetPhy()->SetDlChannel(enb->GetPhy()->GetDlChannel());
    ue->GetPhy()->SetUlChannel(enb->GetPhy()->GetUlChannel());

    networkManager->GetUserEquipmentContainer()->push_back(ue);

    // register UE to the eNodeB
    enb->RegisterUserEquipment(ue);

    // define the channel realization
    MacroCellUrbanAreaChannelRealization *c_dl = new MacroCellUrbanAreaChannelRealization(enb, ue);
    enb->GetPhy()->GetDlChannel()->GetPropagationLossModel()->AddChannelRealization(c_dl);
    MacroCellUrbanAreaChannelRealization *c_ul = new MacroCellUrbanAreaChannelRealization(ue, enb);
    enb->GetPhy()->GetUlChannel()->GetPropagationLossModel()->AddChannelRealization(c_ul);

    ue->StartRandomAccessProcedure(0.000);
  }

  simulator->Run();
}