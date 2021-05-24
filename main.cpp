#include <iostream>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <thread>
#include <mutex>
#include <queue>

#include "gl_frontEnd.h"

void initializeApplication(void);
GridPosition getNewFreePosition(void);
Direction newDirection(Direction forbiddenDir = NUM_DIRECTIONS);
TravelerSegment newTravelerSeg(const TravelerSegment& currentSeg, bool& canAdd);
void generateWalls(void);

SquareType** grid;
std::mutex gridMutex;

AStarNode** defaultAStarGrid;
std::vector<AStarNode**> travelerAStarGrid; //each traveler will need their own copy of aStarGrid
std::vector<std::thread> travelerThreads;
std::vector<AStarNodeSet> aStarNodeSets;

unsigned int numRows = 0;	//	height of the grid
unsigned int numCols = 0;	//	width
unsigned int numTravelers = 1;	//	initial number

std::mutex travelerDoneMutex;
unsigned int numTravelersDone = 0;
std::queue<int> travelersToDelete;

unsigned int numLiveThreads = 0;		//	the number of live traveler threads
std::vector<Traveler> travelerList;
std::mutex travelerListMutex;

GridPosition	exitPos;	//	location of the exit

const int MIN_SLEEP_TIME = 100; //milliseconds
int travelerSleepTime = MIN_SLEEP_TIME;

const int MAX_NUM_MESSAGES = 8;
const int MAX_LENGTH_MESSAGE = 32;
char** message;
time_t launchTime;

std::vector<Direction> getPath(AStarNode& start, AStarNode& end)
{
	std::vector<Direction> path;
	AStarNode& currentNode = end;
	while (!(currentNode == start))
	{
		int colDiff = currentNode.pos.col - currentNode.parent->pos.col;
		int rowDiff = currentNode.pos.row - currentNode.parent->pos.row;
		if (colDiff > 0)
		{
			path.push_back(EAST);
		}
		else if(colDiff < 0)
		{
			path.push_back(WEST);
		}
		else if (rowDiff > 0)
		{
			path.push_back(SOUTH);
		}
		else if (rowDiff < 0)
		{
			path.push_back(NORTH);
		}
		currentNode = *currentNode.parent;
	}
	return path;
}

unsigned int calcualteDistanceCost(const GridPosition& start, const GridPosition& end)
{
	int colDist = (start.col - end.col);
	int rowDist = (start.row - end.row);
	return abs(colDist + rowDist);
}

bool isNodeInClosedSet(const AStarNodeSet& set, const AStarNode& node)
{
	for (const AStarNode* a : set.closed)
	{
		if (*a == node)
		{
			return true;
		}
	}
	return false;
}

bool isNodeInOpenSet(const AStarNodeSet& set, const AStarNode& node)
{
	for (const AStarNode* a : set.open)
	{
		if (*a == node)
		{
			return true;
		}
	}
	return false;
}

int getLowestCostNodeIndex(const AStarNodeSet& set)
{
	unsigned int lowest = MAXUINT32;
	unsigned int index = 0;
	for (int i = 0; i < set.open.size(); i++)
	{
		unsigned int fCost = set.open[i]->gCost + set.open[i]->hCost;
		if (fCost < lowest)
		{
			lowest = fCost;
			index = i;
		}
	}
	return index;
}

unsigned int calculateGCost(const GridPosition& end, AStarNode& node)
{
	
	unsigned int cost = 0;
	if (node.parent)
	{
		cost = calcualteDistanceCost(end, node.parent->pos);
		return node.parent->gCost + cost;
	}
	return cost;
}

unsigned int calculateHCost(const GridPosition& current, const GridPosition& end)
{
	int colDist = (current.col - end.col);
	int rowDist = (current.row - end.row);
	unsigned int endDist = abs(colDist) + abs(rowDist);
	return endDist;
}

bool isPosInBounds(const GridPosition& pos)
{
	if (pos.col < 0 || pos.col >= numCols || pos.row < 0 || pos.row >= numRows)
	{
		return false;
	}
	return true;
}

std::vector<GridPosition> getNeighbors(const GridPosition& currPos)
{
	std::vector<GridPosition> neighbors;
	GridPosition testPos = currPos;
	testPos.col++;
	gridMutex.lock();
	if (isPosInBounds(testPos))
	{
		if (grid[testPos.row][testPos.col] == FREE_SQUARE || grid[testPos.row][testPos.col] == EXIT)
		{
			neighbors.push_back(testPos);
		}
	}
	testPos.col -= 2;
	if (isPosInBounds(testPos))
	{
		if (grid[testPos.row][testPos.col] == FREE_SQUARE || grid[testPos.row][testPos.col] == EXIT)
		{
			neighbors.push_back(testPos);
		}
	}
	testPos = currPos;
	testPos.row++;
	if (isPosInBounds(testPos))
	{
		if (grid[testPos.row][testPos.col] == FREE_SQUARE || grid[testPos.row][testPos.col] == EXIT)
		{
			neighbors.push_back(testPos);
		}
	}
	testPos.row -= 2;
	if (isPosInBounds(testPos))
	{
		if (grid[testPos.row][testPos.col] == FREE_SQUARE || grid[testPos.row][testPos.col] == EXIT)
		{
			neighbors.push_back(testPos);
		}
	}
	gridMutex.unlock();
	return neighbors;
}

std::vector<Direction> getShortestPath(const GridPosition& start, const GridPosition& finish, AStarNode** aStarGrid, AStarNodeSet& set)
{
	set.closed.clear();
	set.open.clear();
	AStarNode& startNode = aStarGrid[start.row][start.col];
	startNode.hCost = calculateHCost(start, finish);
	startNode.gCost = 0;
	startNode.parent = nullptr;

	set.open.push_back(&aStarGrid[start.row][start.col]);


	while (set.open.size() > 0)
	{
		int currentIndex = getLowestCostNodeIndex(set);
		AStarNode* currentNode = set.open[currentIndex];
		set.closed.push_back(currentNode);
		set.open.erase(set.open.begin() + currentIndex);

		if (currentNode->pos == finish)
		{
			return(getPath(startNode, *currentNode));
		}
		std::vector<GridPosition> neighbors = getNeighbors(currentNode->pos);
		for (GridPosition g : neighbors)
		{
			AStarNode& neighborNode = aStarGrid[g.row][g.col];

			if (isNodeInClosedSet(set, neighborNode))
			{
				continue;
			}

			unsigned int newGcost = currentNode->gCost + 1;
			bool inOpenSet = isNodeInOpenSet(set, neighborNode);
			if (newGcost < neighborNode.gCost || !inOpenSet);
			{
				neighborNode.gCost = newGcost;
				neighborNode.hCost = calculateHCost(neighborNode.pos, exitPos);
				neighborNode.parent = currentNode;
				if (!inOpenSet)
				{
					set.open.push_back(&aStarGrid[g.row][g.col]);
				}
			}
		}
	}

	std::vector<Direction> path;
	return path;
}


bool moveTraveler(Traveler& traveler, Direction dir)
{
	GridPosition g = { traveler.segmentList[0].row, traveler.segmentList[0].col };
	switch (dir)
	{
	case NORTH:
	{
		g.row -= 1;
	}
	break;
	case SOUTH:
	{
		g.row += 1;
	}
	break;
	case EAST:
	{
		g.col += 1;
	}
	break;
	case WEST:
	{
		g.col -= 1;
	}
	break;
	}

	if (g.col < 0 || g.row < 0 || g.col >= numCols || g.row >= numRows)
	{
		return false;
	}

	Direction lastDir = dir;
	GridPosition lastPos = g;
	gridMutex.lock();
	if (grid[g.row][g.col] == EXIT)
	{
		travelerListMutex.lock();

		GridPosition pos;
		for (int i = 0; i < traveler.segmentList.size(); i++)
		{
			pos = { traveler.segmentList[i].row, traveler.segmentList[i].col };
			grid[pos.row][pos.col] = FREE_SQUARE;
		}
		traveler.segmentList.clear();
		travelerListMutex.unlock();
		gridMutex.unlock();
		return true;
	}

	if (grid[g.row][g.col] != FREE_SQUARE)
	{

		gridMutex.unlock();
		return false;
	}
	gridMutex.unlock();

	travelerListMutex.lock();
	for (int i = 0; i < traveler.segmentList.size(); i++)
	{
		grid[lastPos.row][lastPos.col] = TRAVELER;
		GridPosition tempPos = { traveler.segmentList[i].row, traveler.segmentList[i].col };
		Direction tempDir = traveler.segmentList[i].dir;
		traveler.segmentList[i].dir = lastDir;
		traveler.segmentList[i].col = lastPos.col;
		traveler.segmentList[i].row = lastPos.row;
		lastDir = tempDir;
		lastPos = tempPos;
		grid[lastPos.row][lastPos.col] = FREE_SQUARE;
		
	}
	travelerListMutex.unlock();
	return true;
}

void drawTravelers(void)
{
	for (unsigned int k=0; k<travelerList.size(); k++)
	{
		drawTraveler(travelerList[k]);

		travelerDoneMutex.lock();
		while (travelersToDelete.size() > 0)
		{
			int index = travelersToDelete.front();
			travelerThreads[index].join();
			travelersToDelete.pop();
			numLiveThreads--;
		}
		travelerDoneMutex.unlock();
	}
}

void updateMessages(void)
{
	unsigned int numMessages = 2;
	sprintf(message[0], "We created %d travelers", numTravelers);
	sprintf(message[1], "%d travelers solved the maze", numTravelersDone);
	
	drawMessages(numMessages, message);
}

void handleKeyboardEvent(unsigned char c, int x, int y)
{
	switch (c)
	{
		//	'esc' to quit
		case 27:
			exit(0);
			break;
		//	slowdown
		case 'w':
			slowdownTravelers();
			break;
		//	speedup
		case 's':
			speedupTravelers();
			break;
		default:
			break;
	}
}

void speedupTravelers(void)
{
	//	decrease sleep time by 20%, but don't get too small
	int newSleepTime = (8 * travelerSleepTime) / 10;
	
	if (newSleepTime > MIN_SLEEP_TIME)
	{
		travelerSleepTime = newSleepTime;
	}
}

void slowdownTravelers(void)
{
	//	increase sleep time by 20%
	travelerSleepTime = (12 * travelerSleepTime) / 10;
}

//function that runs each traveler thread runs
void travelerLoop(Traveler& traveler, AStarNode** grid)
{
	std::vector<Direction> path;
	AStarNodeSet set;
	GridPosition currPos = { traveler.segmentList[0].row, traveler.segmentList[0].col };
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	while (1)
	{
		if (path.size() == 0)
		{
			for (int row = 0; row < numRows; row++)
			{
				memcpy(grid[row], defaultAStarGrid[row], sizeof(AStarNode)*numCols);
			}

			path = (getShortestPath(currPos, exitPos, grid, set));
		}
		else
		{
			if (moveTraveler(travelerList[traveler.index], path.back()))
			{
				path.pop_back();
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(travelerSleepTime));
			if (path.size() == 0)
			{
				travelerDoneMutex.lock();
				numTravelersDone++;
				travelersToDelete.push(traveler.index);
				travelerDoneMutex.unlock();
				break;
			}
		}
	}
	return;
}

int main(int argc, char** argv)
{
	if (argc != 4)
	{
		std::cout << "incorrect number of arguments" << std::endl;
		return 0;
	}
	else
	{
		numRows = atoi(argv[1]);
		numCols = atoi(argv[2]);
		numTravelers = atoi(argv[3]);
		numLiveThreads = numTravelers;
	}

	numTravelersDone = 0;

	initializeFrontEnd(argc, argv);
	
	initializeApplication();

	launchTime = time(NULL);

	for (int i = 0; i < travelerList.size(); i++)
	{
		travelerThreads.push_back(std::thread(travelerLoop, travelerList[i], travelerAStarGrid[i]));
	}

	glutMainLoop();

	for (unsigned int i = 0; i < numRows; i++)
	{
		free(grid[i]);
		free(defaultAStarGrid[i]);
	}

	free(grid);
	free(defaultAStarGrid);

	for (unsigned int t = 0; t < numTravelers; t++)
	{
		for (unsigned int i = 0; i < numRows; i++)
		{
			free(travelerAStarGrid[t][i]);
		}
		free(travelerAStarGrid[t]);
	}

	for (int k=0; k<MAX_NUM_MESSAGES; k++)
		free(message[k]);
	free(message);
	
	return 0;
}

void initializeApplication(void)
{
	//	Allocate the grid
	grid = new SquareType*[numRows];
	defaultAStarGrid = new AStarNode*[numRows];
	for (unsigned int i=0; i<numRows; i++)
	{
		defaultAStarGrid[i] = new AStarNode[numCols];
		grid[i] = new SquareType[numCols];
		for (unsigned int j = 0; j < numCols; j++)
		{
			grid[i][j] = FREE_SQUARE;
			defaultAStarGrid[i][j] = AStarNode{ {i, j}, nullptr, MAXUINT32, MAXUINT32 };
		}
	}

	//allocate each travlers own AstarGrid
	for (unsigned int t = 0; t < numTravelers; t++)
	{
		travelerAStarGrid.push_back(new AStarNode*[numRows]);
		for (unsigned int j = 0; j < numRows; j++)
		{

			travelerAStarGrid[t][j] = new AStarNode[numCols];
		}
	}

	message = (char**) malloc(MAX_NUM_MESSAGES*sizeof(char*));
	for (unsigned int k=0; k<MAX_NUM_MESSAGES; k++)
		message[k] = (char*) malloc((MAX_LENGTH_MESSAGE+1)*sizeof(char));
		
	srand((unsigned int) time(NULL));

	exitPos = getNewFreePosition();
	grid[exitPos.row][exitPos.col] = EXIT;

	generateWalls();
	
	//	Initialize traveler info structs
	float** travelerColor = createTravelerColors(numTravelers);
	for (unsigned int k=0; k<numTravelers; k++) {
		GridPosition pos = getNewFreePosition();
		Direction dir = static_cast<Direction>(rand() % NUM_DIRECTIONS);
		TravelerSegment seg = {pos.row, pos.col, dir};
		Traveler traveler;
		traveler.segmentList.push_back(seg);
		grid[pos.row][pos.col] = TRAVELER;

		unsigned int numAddSegments = (rand() % 6)+1;
		TravelerSegment currSeg = traveler.segmentList[0];
		bool canAddSegment = true;
		for (unsigned int s=0; s<numAddSegments && canAddSegment; s++)
		{
			TravelerSegment newSeg = newTravelerSeg(currSeg, canAddSegment);
			if (canAddSegment)
			{
				traveler.segmentList.push_back(newSeg);
				currSeg = newSeg;
			}
		}
		
		for (unsigned int c=0; c<4; c++)
			traveler.rgba[c] = travelerColor[k][c];
		traveler.index = travelerList.size();
		travelerList.push_back(traveler);
		
	}
	
	//	free array of colors
	for (unsigned int k=0; k<numTravelers; k++)
		delete []travelerColor[k];
	delete []travelerColor;
}

GridPosition getNewFreePosition(void)
{
	GridPosition pos{ 0,0 };

	bool noGoodPos = true;
	while (noGoodPos)
	{
		unsigned int row = rand() % numRows;
		unsigned int col = rand() % numCols;
		if (grid[row][col] == FREE_SQUARE)
		{
			pos.row = row;
			pos.col = col;
			noGoodPos = false;
		}
	}
	return pos;
}

Direction newDirection(Direction forbiddenDir)
{
	bool noDir = true;

	Direction dir = NUM_DIRECTIONS;
	while (noDir)
	{
		dir = static_cast<Direction>(rand() % NUM_DIRECTIONS);
		noDir = (dir==forbiddenDir);
	}
	return dir;
}

TravelerSegment newTravelerSeg(const TravelerSegment& currentSeg, bool& canAdd)
{
	TravelerSegment newSeg;
	switch (currentSeg.dir)
	{
		case NORTH:
			if (	currentSeg.row < numRows-1 &&
					grid[currentSeg.row+1][currentSeg.col] == FREE_SQUARE)
			{
				newSeg.row = currentSeg.row+1;
				newSeg.col = currentSeg.col;
				newSeg.dir = newDirection(SOUTH);
				grid[newSeg.row][newSeg.col] = TRAVELER;
				canAdd = true;
			}
			//	no more segment
			else
				canAdd = false;
			break;

		case SOUTH:
			if (	currentSeg.row > 0 &&
					grid[currentSeg.row-1][currentSeg.col] == FREE_SQUARE)
			{
				newSeg.row = currentSeg.row-1;
				newSeg.col = currentSeg.col;
				newSeg.dir = newDirection(NORTH);
				grid[newSeg.row][newSeg.col] = TRAVELER;
				canAdd = true;
			}
			//	no more segment
			else
				canAdd = false;
			break;

		case WEST:
			if (	currentSeg.col < numCols-1 &&
					grid[currentSeg.row][currentSeg.col+1] == FREE_SQUARE)
			{
				newSeg.row = currentSeg.row;
				newSeg.col = currentSeg.col+1;
				newSeg.dir = newDirection(EAST);
				grid[newSeg.row][newSeg.col] = TRAVELER;
				canAdd = true;
			}
			//	no more segment
			else
				canAdd = false;
			break;

		case EAST:
			if (	currentSeg.col > 0 &&
					grid[currentSeg.row][currentSeg.col-1] == FREE_SQUARE)
			{
				newSeg.row = currentSeg.row;
				newSeg.col = currentSeg.col-1;
				newSeg.dir = newDirection(WEST);
				grid[newSeg.row][newSeg.col] = TRAVELER;
				canAdd = true;
			}
			//	no more segment
			else
				canAdd = false;
			break;
		
		default:
			canAdd = false;
	}
	
	return newSeg;
}

void generateWalls(void)
{
	const unsigned int NUM_WALLS = (numCols+numRows)/4;

	const unsigned int MIN_WALL_LENGTH = 3;
	const unsigned int MAX_HORIZ_WALL_LENGTH = numCols / 3;
	const unsigned int MAX_VERT_WALL_LENGTH = numRows / 3;
	const unsigned int MAX_NUM_TRIES = 20;

	bool goodWall = true;
	
	//	Generate the vertical walls
	for (unsigned int w=0; w< NUM_WALLS; w++)
	{
		goodWall = false;
		
		//	Case of a vertical wall
		if (rand() %2)
		{
			//	I try a few times before giving up
			for (unsigned int k=0; k<MAX_NUM_TRIES && !goodWall; k++)
			{
				//	let's be hopeful
				goodWall = true;
				
				//	select a column index
				unsigned int HSP = numCols/(NUM_WALLS/2+1);
				unsigned int col = (1+ rand()%(NUM_WALLS/2-1))*HSP;
				unsigned int length = MIN_WALL_LENGTH + rand()%(MAX_VERT_WALL_LENGTH-MIN_WALL_LENGTH+1);
				
				//	now a random start row
				unsigned int startRow = rand()%(numRows-length);
				for (unsigned int row=startRow, i=0; i<length && goodWall; i++, row++)
				{
					if (grid[row][col] != FREE_SQUARE)
						goodWall = false;
				}
				
				//	if the wall first, add it to the grid
				if (goodWall)
				{
					for (unsigned int row=startRow, i=0; i<length && goodWall; i++, row++)
					{
						grid[row][col] = WALL;
					}
				}
			}
		}
		// case of a horizontal wall
		else
		{
			goodWall = false;
			
			//	I try a few times before giving up
			for (unsigned int k=0; k<MAX_NUM_TRIES && !goodWall; k++)
			{
				//	let's be hopeful
				goodWall = true;
				
				//	select a column index
				unsigned int VSP = numRows/(NUM_WALLS/2+1);
				unsigned int row = (1+ rand()%(NUM_WALLS/2-1))*VSP;
				unsigned int length = MIN_WALL_LENGTH + rand()%(MAX_HORIZ_WALL_LENGTH-MIN_WALL_LENGTH+1);
				
				//	now a random start row
				unsigned int startCol = rand()%(numCols-length);
				for (unsigned int col=startCol, i=0; i<length && goodWall; i++, col++)
				{
					if (grid[row][col] != FREE_SQUARE)
						goodWall = false;
				}
				
				//	if the wall first, add it to the grid
				if (goodWall)
				{
					for (unsigned int col=startCol, i=0; i<length && goodWall; i++, col++)
					{
						grid[row][col] = WALL;
					}
				}
			}
		}
	}
}