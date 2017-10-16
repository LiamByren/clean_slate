/**
         (                                      
   (     )\ )                                   
 ( )\   (()/(   (    ) (        (        (  (   
 )((_)   /(_)) ))\( /( )(   (   )\  (    )\))(  
((_)_   (_))  /((_)(_)|()\  )\ |(_) )\ )((_))\  
 / _ \  | |  (_))((_)_ ((_)_(_/((_)_(_/( (()(_) 
| (_) | | |__/ -_) _` | '_| ' \)) | ' \)) _` |  
 \__\_\ |____\___\__,_|_| |_||_||_|_||_|\__, |  
                                        |___/   

Refer to Watkins, Christopher JCH, and Peter Dayan. "Q-learning." Machine learning 8. 3-4 (1992): 279-292
for a detailed discussion on Q Learning
*/
#include "CQLearningController.h"


CQLearningController::CQLearningController(HWND hwndMain):
	CDiscController(hwndMain),
	_grid_size_x(CParams::WindowWidth / CParams::iGridCellDim + 1),
	_grid_size_y(CParams::WindowHeight / CParams::iGridCellDim + 1)
{
}
/**
 The update method should allocate a Q table for each sweeper (this can
 be allocated in one shot - use an offset to store the tables one after the other)

 You can also use a boost multiarray if you wish
*/
void CQLearningController::InitializeLearningAlgorithm(void)
{
	//TODO
	//int size = m_NumSweepers* _grid_size_x * _grid_size_y*4;
	qtable = vector<vector<vector<vector<double>>>>(m_NumSweepers,
				vector<vector<vector<double>>>(_grid_size_x,
					vector<vector<double>>(_grid_size_y,
						vector<double>(4,0))));
		

}
/**
 The immediate reward function. This computes a reward upon achieving the goal state of
 collecting all the mines on the field. It may also penalize movement to encourage exploring all directions and 
 of course for hitting supermines/rocks!
*/
double CQLearningController::R(uint x,uint y, uint sweeper_no){
	//TODO: roll your own here!
	int objectHit = m_vecSweepers[sweeper_no]->CheckForObject(m_vecObjects, CParams::dMineScale);

	double reward = -5.0f;
	if (objectHit >= 0) {
		switch (m_vecObjects[objectHit]->getType()) {
		case CDiscCollisionObject::Mine:
			reward = 1000.0f;
			break;
		case CDiscCollisionObject::SuperMine:
			reward = -300.0f;
			break;
		case CDiscCollisionObject::Rock:
			reward = -300.0f;
			break;
		}
	}
	return reward;

	}
/**
The update method. Main loop body of our Q Learning implementation
See: Watkins, Christopher JCH, and Peter Dayan. "Q-learning." Machine learning 8. 3-4 (1992): 279-292
*/
bool CQLearningController::Update(void)
{

	//m_vecSweepers is the array of minesweepers
	//everything you need will be m_[something] ;)
	uint cDead = std::count_if(m_vecSweepers.begin(),
							   m_vecSweepers.end(),
						       [](CDiscMinesweeper * s)->bool{
								return s->isDead();
							   });
	if (cDead == CParams::iNumSweepers){
		printf("All dead ... skipping to next iteration\n");
		m_iTicks = CParams::iNumTicks;
	}

	for (uint sw = 0; sw < CParams::iNumSweepers; ++sw){
		if (m_vecSweepers[sw]->isDead()) continue;
		/**
		Q-learning algorithm according to:
		Watkins, Christopher JCH, and Peter Dayan. "Q-learning." Machine learning 8. 3-4 (1992): 279-292
		*/

		//1:::Observe the current state:
		SVector2D<int> pos = m_vecSweepers[sw]->Position();
		pos /= CParams::iGridCellDim;

		/*
		if (sw == 5) {
			std::string s = std::to_string(pos.x) + " " + std::to_string(pos.y)+ "\n";
			char const *pchar = s.c_str();
			OutputDebugString(pchar);
		}*/

		//2:::Select action with highest historic return:
		int highestreturn = RandInt(0, 3);
		int highestvalue = 0;
		for (int i = 0; i < 4; i++) {
			if (qtable[sw][pos.x][pos.y][i] > highestvalue) {
				highestreturn = i;
				highestvalue = qtable[sw][pos.x][pos.y][i];
			}
		}

		switch (highestreturn) {
		case 0:
			m_vecSweepers[sw]->setRotation(EAST);
			break;
		case 1:
			m_vecSweepers[sw]->setRotation(NORTH);
			break;
		case 2:
			m_vecSweepers[sw]->setRotation(WEST);
			break;
		case 3:
			m_vecSweepers[sw]->setRotation(SOUTH);
			break;
		}
		//TODO
		//now call the parents update, so all the sweepers fulfill their chosen action
		//CDiscController::Update();
	}
	
	CDiscController::Update(); //call the parent's class update. Do not delete this.

							   
	for (uint sw = 0; sw < CParams::iNumSweepers; ++sw){
		if (m_vecSweepers[sw]->isDead()) continue;

		SVector2D<int> prevpos = m_vecSweepers[sw]->PrevPosition();
		prevpos /= CParams::iGridCellDim;

		SVector2D<int> pos = m_vecSweepers[sw]->Position();
		pos /= CParams::iGridCellDim;

		int move = (int)m_vecSweepers[sw]->getRotation();

		double maxEW = max(qtable[sw][pos.x][pos.y][0], qtable[sw][pos.x][pos.y][2]);
		double maxNS = max(qtable[sw][pos.x][pos.y][1], qtable[sw][pos.x][pos.y][3]);
		double maxF = max(maxEW, maxNS);

			double learingrate = 0.5;
			double discountfactor = 0.8;

		qtable[sw][prevpos.x][prevpos.y][move] += learingrate*(R(pos.x, pos.y, sw)+discountfactor *maxF - abs(qtable[sw][prevpos.x][prevpos.y][move]));
		//TODO:compute your indexes.. it may also be necessary to keep track of the previous state
		//3:::Observe new state:
		//TODO
		//4:::Update _Q_s_a accordingly:
		//TODO


	}
	return true;
}

CQLearningController::~CQLearningController(void)
{
	//TODO: dealloc stuff here if you need to	
}
