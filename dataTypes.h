#pragma once

#include <vector>
#include <thread>

typedef enum Direction {
							NORTH = 0,
							WEST,
							SOUTH,
							EAST,
							//
							NUM_DIRECTIONS
} Direction;

//	Grid square types for this simulation
typedef enum SquareType {
							FREE_SQUARE,
							EXIT,
							WALL,
							TRAVELER,
							NUM_SQUARE_TYPES
} SquareType;

//	Data type to store the position of things on the grid
typedef struct GridPosition {
								unsigned int row;
								unsigned int col;
} GridPosition;

inline bool operator==(const GridPosition& left, const GridPosition& right)
{
	if (left.row == right.row && left.col == right.col)
	{
		return true;
	}
	return false;
}

//	Data type to store the position and orientation of a traveler's segment
typedef struct TravelerSegment {
								unsigned int row = 0;
								unsigned int col = 0;
								Direction dir = NORTH;
} TravelerSegment;

//Traveler info
typedef struct Traveler {
	unsigned int index;
	float rgba[4];
	std::vector<TravelerSegment> segmentList;
} Traveler;

//Astar Node
typedef struct AStarNode {
	GridPosition pos;
	AStarNode* parent;
	unsigned int gCost;
	unsigned int hCost;

	unsigned int calculateFCost()
	{
		return gCost + hCost;
	}

} AStarNode;

inline bool operator==(const AStarNode& left , const AStarNode& right)
{
	if (left.pos == right.pos)
	{
		return true;
	}
	return false;
}

//stores open and closed AStar nodes
typedef struct AStarNodeSet
{
	std::vector<AStarNode*> open;
	std::vector<AStarNode*> closed;
} AStarNodeSet;
