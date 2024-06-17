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

static void Scenario_03(int raMethod, int seed, int nbUE_HTC, int nbUE_MTC, int RACHIndex, int preambleTransMax, int RARWindowSize, int backoffIndex, int nbPreambles, double acbAP, double acbAD) {
  int idUniq = 0;
	int iteratorSlot, iteratorDevice = 0, i;

  // Create COMPONENT MANAGERS
  Simulator *simulator = Simulator::Init();
  FrameManager *frameManager = FrameManager::Init();
  NetworkManager *networkManager = NetworkManager::Init();
  FlowsManager *flowsManager = FlowsManager::Init();
  InformationManager *informationManager = InformationManager::Init();
  Random *rndGenerator = Random::Init(seed);
	BetaArrival *betaArrival = BetaArrival::Init();

  // Main CONFIGURATIONS
  informationManager->ra_Method = raMethod;
  informationManager->mac_ContentionResolutionTimer = 48;
  informationManager->numberOfRA_Preambles = nbPreambles;
  informationManager->preambleTransMax = preambleTransMax;
  informationManager->ra_PRACH_MaskIndex = RACHIndex;
  informationManager->ra_ResponseWindowSize = RARWindowSize;
  informationManager->backoffIndicatorGeneral = backoffIndex;
  informationManager->ac_BarringFactor = acbAP;
  informationManager->ac_BarringTime = acbAD;

  // Create CHANNELS
  LteChannel *dlCh = new LteChannel();
  LteChannel *ulCh = new LteChannel();

  // Create SPECTRUM
  BandwidthManager *spectrum = new BandwidthManager(5, 5, 0, 0);

  // Create CELL
  Cell *cell = networkManager->CreateCell(idUniq++, 0.5, 0.050, 0, 0);

  // Create ENodeB
  ENodeB *enb = networkManager->CreateEnodeb(idUniq++, cell, 0, 0, dlCh, ulCh, spectrum);

  //user equipments are distributed uniformly into a cell
  vector<CartesianCoordinates*> *positions = GetUniformUsersDistribution(cell->GetIdCell(), nbUE_HTC + nbUE_MTC);

  double posX = 0;
  double posY = 0;

  string _file(path + "src/arrival/probability-6-10.txt");
	vector<TimeArrival*> *arrivals = betaArrival->buildArrival(6, 10, nbUE_HTC + nbUE_MTC, _file);

	//std::cout << "Vector size " << arrivals->size() << std::endl;

	for (iteratorSlot = 0; iteratorSlot < arrivals->size(); iteratorSlot++) {

		//std::cout << "UE devices " << arrivals->at(iteratorSlot)->getNumberArrivals() << " arrival at " << arrivals->at(iteratorSlot)->getTimeArrivals() << std::endl;

  	//Create UE MTC devices
	  for (i = 0; i < arrivals->at(iteratorSlot)->getNumberArrivals(); i++) {
  	  posX = positions->at(iteratorDevice)->GetCoordinateX();
	    posY = positions->at(iteratorDevice)->GetCoordinateY();

  	  //std::cout << "Created UE " << idUniq << " on " << posX << " " << posY << std::endl;
      //std::cout << posY << ",";
	    UserEquipment *ue = new UserEquipment(idUniq++, posX, posY, 0, 0, cell, enb, false, Mobility::CONSTANT_POSITION);

	    ue->SetDeviceType(0);
	    ue->SetDeviceAccessClass(Random::Init()->Uniform(0, 9));

  	  ue->GetProtocolStack()->GetMacEntity()->SetQCI(1, 2, 100, true, 3);

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

			iteratorDevice++;

  	  ue->StartRandomAccessProcedure(arrivals->at(iteratorSlot)->getTimeArrivals());
  	}
	}

	arrivals->clear();

  simulator->Run();
}
