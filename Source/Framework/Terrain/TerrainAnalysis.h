/******************************************************************************/
/*!
\file		TerrainAnalysis.h
\project	CS380/CS580 AI Framework
\author		Dustin Holmes
\summary	Terrain analysis function declarations

Copyright (C) 2018 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
*/
/******************************************************************************/

#pragma once

// forward declarations
class Agent;
template <typename T>
class MapLayer;

struct Line
{
	Vec2 p0;
	Vec2 p1;
};

float distance_to_closest_wall(int row, int col);
bool is_clear_path(int row0, int col0, int row1, int col1);
void analyze_openness(MapLayer<float>& layer);
void analyze_visibility(MapLayer<float>& layer);
void analyze_visible_to_cell(MapLayer<float>& layer, int row, int col);
void analyze_agent_vision(MapLayer<float>& layer, const Agent* agent);
void propagate_solo_occupancy(MapLayer<float>& layer, float decay, float growth);
void propagate_dual_occupancy(MapLayer<float>& layer, float decay, float growth);
void normalize_solo_occupancy(MapLayer<float>& layer);
void normalize_dual_occupancy(MapLayer<float>& layer);

float ApplyDecayFromNeighbors(int row, int col, float decay, MapLayer<float>& layer);
bool isDiagonalWalkable(int inStartRow, int inStartCol, int inNeighborRow, int inNeighborColl);
int ComputeNumberOfVisibleCells(int row, int col);
float GridPosDistance(const GridPos& inStart, const GridPos& inEnd);

void enemy_field_of_view(MapLayer<float>& layer, float angle, float closeDistance, float occupancyValue, AStarAgent* enemy);
bool enemy_find_player(MapLayer<float>& layer, AStarAgent* enemy, Agent* player);
bool enemy_seek_player(MapLayer<float>& layer, AStarAgent* enemy);