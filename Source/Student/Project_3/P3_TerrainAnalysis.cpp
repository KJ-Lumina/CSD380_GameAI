#include <pch.h>
#include "Terrain/TerrainAnalysis.h"
#include "Terrain/MapMath.h"
#include "Agent/AStarAgent.h"
#include "Terrain/MapLayer.h"
#include "Projects/ProjectThree.h"

#include <iostream>

bool ProjectThree::implemented_fog_of_war() const // extra credit
{
    return false;
}

float GridPosDistance(const GridPos& inStart, const GridPos& inEnd)
{
	return sqrtf(static_cast<float>((inStart.row - inEnd.row) * (inStart.row - inEnd.row) + (inStart.col - inEnd.col) * (inStart.col - inEnd.col)));
}

bool isDiagonalWalkable(int inStartRow, int inStartCol ,int inNeighborRow, int inNeighborCol)
{
	//Determine which direction the neighbor is in relation to the start
	if (inNeighborRow < inStartRow)
	{
		if (inNeighborCol < inStartCol)
		{
			//Northwest
			if (terrain->is_wall(GridPos{ inStartRow - 1, inStartCol }) || terrain->is_wall(GridPos{ inStartRow, inStartCol - 1 }))
			{
				return false;
			}
		}
		else if (inNeighborCol > inStartCol)
		{
			//Northeast
			if (terrain->is_wall(GridPos{ inStartRow - 1, inStartCol }) || terrain->is_wall(GridPos{ inStartRow, inStartCol + 1 }))
			{
				return false;
			}
		}
	}
	else if (inNeighborRow > inStartRow)
	{
		if (inNeighborCol < inStartCol)
		{
			//Southwest
			if (terrain->is_wall(GridPos{ inStartRow + 1, inStartCol }) || terrain->is_wall(GridPos{ inStartRow, inStartCol - 1 }))
			{
				return false;
			}
		}
		else if (inNeighborCol > inStartCol)
		{
			//Southeast
			if (terrain->is_wall(GridPos{ inStartRow + 1, inStartCol }) || terrain->is_wall(GridPos{ inStartRow, inStartCol + 1 }))
			{
				return false;
			}
		}
	}

    return true;
}

float ApplyDecayFromNeighbors(int row, int col, float decay, MapLayer<float>& layer)
{
    float result = -FLT_MAX;
	float originalValue = layer.get_value(row, col);
	for (int i = -1; i < 2; ++i)
	{
		for (int j = -1; j < 2; ++j)
		{
			GridPos pos{ row + i, col + j };

			if (terrain->is_valid_grid_position(pos) && terrain->is_wall(pos))
			{
                float newValue = originalValue * expf(-1 * decay);
                result = std::max(result, newValue);
			}
		}
	}

	return result;
}

float distance_to_closest_wall(int row, int col)
{
    /*
        Check the euclidean distance from the given cell to every other wall cell,
        with cells outside the map bounds treated as walls, and return the smallest
        distance.  Make use of the is_valid_grid_position and is_wall member
        functions in the global terrain to determine if a cell is within map bounds
        and a wall, respectively.
    */

    std::vector<float> walls;
    const GridPos currentPos{ row, col };

    // WRITE YOUR CODE HERE
    for(int i = -1; i < terrain->get_map_height() + 1; ++i)
    {
	    for(int j = -1; j < terrain->get_map_width() + 1; ++j)
	    {
			GridPos pos { i,j };

            if(!terrain->is_valid_grid_position(pos) || terrain->is_wall(pos))
            {
                float distance = GridPosDistance(pos, currentPos);
				walls.push_back(distance);
            }
	    }
    }

	std::sort(walls.begin(), walls.end());
    
    return walls.front(); // REPLACE THIS
}

bool is_clear_path(int row0, int col0, int row1, int col1)
{
    /*
        Two cells (row0, col0) and (row1, col1) are visible to each other if a line
        between their centerpoints doesn't intersect the four boundary lines of every
        wall cell.  You should puff out the four boundary lines by a very tiny amount
        so that a diagonal line passing by the corner will intersect it.  Make use of the
        line_intersect helper function for the intersection test and the is_wall member
        function in the global terrain to determine if a cell is a wall or not.
    */

    // WRITE YOUR CODE HERE

	// Determine the bounding box of the two points TODO: Find out how to use this
    int min_row = std::min(row0, row1);
    int max_row = std::max(row0, row1);
    int min_col = std::min(col0, col1);
    int max_col = std::max(col0, col1);

    Vec3 x = terrain->get_world_position({ 1,0 });
    Vec3 y = terrain->get_world_position({ 0,0 });

	const float worldPosUnitDistance = Vec2::Distance({ x.x, x.z }, { y.x, y.z });

	Vec3 worldPos0 = terrain->get_world_position({ row0, col0 });
	Vec3 worldPos1 = terrain->get_world_position({ row1, col1 });

	const Line line1{ { worldPos0.x, worldPos0.z }, { worldPos1.x,worldPos1.z } };

    for(int i = 0; i < terrain->get_map_height(); ++i)
    {
        for (int j = 0; j < terrain->get_map_width(); ++j)
        {
            if (terrain->is_wall({ i,j }))
            {
                // Find the four corners of the wall
				const Vec3 wallWorldPos = terrain->get_world_position({ i,j });

				Vec2 topLeft{ wallWorldPos.x - worldPosUnitDistance / 2.0f , wallWorldPos.z + worldPosUnitDistance / 2.0f };
				Vec2 bottomLeft{ wallWorldPos.x - worldPosUnitDistance / 2.0f, wallWorldPos.z - worldPosUnitDistance / 2.0f };
				Vec2 topRight{ wallWorldPos.x + worldPosUnitDistance / 2.0f, wallWorldPos.z + worldPosUnitDistance / 2.0f };
				Vec2 bottomRight{ wallWorldPos.x + worldPosUnitDistance / 2.0f, wallWorldPos.z - worldPosUnitDistance / 2.0f };

				if (line_intersect(line1.p0, line1.p1, topLeft, bottomLeft ) || line_intersect(line1.p0, line1.p1, bottomLeft, bottomRight) 
                    || line_intersect(line1.p0, line1.p1, bottomRight, topRight) || line_intersect(line1.p0, line1.p1, topRight, topLeft))
				{
					return false;
				}
            }
        }
    }

    return true;
}

int ComputeNumberOfVisibleCells(int row, int col)
{
    int count = 0;
    for (int i = 0; i < terrain->get_map_height(); ++i)
    {
        for (int j = 0; j < terrain->get_map_width(); ++j)
        {
            if (i == row && j == col)
                continue;

            if(is_clear_path(row, col, i, j))
            {
                ++count;
            }
        }
    }
    return count;
}

void analyze_openness(MapLayer<float> &layer)
{
    /*
        Mark every cell in the given layer with the value 1 / (d * d),
        where d is the distance to the closest wall or edge.  Make use of the
        distance_to_closest_wall helper function.  Walls should not be marked.
    */

    // WRITE YOUR CODE HERE

    for (int i = 0; i < terrain->get_map_height(); ++i)
    {
        for (int j = 0; j < terrain->get_map_width(); ++j)
        {
			if (!terrain->is_valid_grid_position({ i,j }) || terrain->is_wall({ i,j }))
				continue;

            const float dist = distance_to_closest_wall(i, j);
            layer.set_value(GridPos{ i,j }, 1/ (dist * dist));
        }
    }
}

void analyze_visibility(MapLayer<float> &layer)
{
    /*
        Mark every cell in the given layer with the number of cells that
        are visible to it, divided by 160 (a magic number that looks good).  Make sure
        to cap the value at 1.0 as well.

        Two cells are visible to each other if a line between their centerpoints doesn't
        intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
        helper function.
    */

    // WRITE YOUR CODE HERE

    for(int i = 0; i < terrain->get_map_height(); ++i)
    {
	    for(int j = 0; j < terrain->get_map_width(); ++j)
	    {
			if (!terrain->is_valid_grid_position({ i,j }) || terrain->is_wall({ i,j }))
				continue;

            int count = ComputeNumberOfVisibleCells(i, j);
			layer.set_value(GridPos{ i,j }, std::min(1.0f, static_cast<float>(count) / 160.0f));
	    }
    }
}

void analyze_visible_to_cell(MapLayer<float> &layer, int row, int col)
{
    /*
        For every cell in the given layer mark it with 1.0
        if it is visible to the given cell, 0.5 if it isn't visible but is next to a visible cell,
        or 0.0 otherwise.

        Two cells are visible to each other if a line between their centerpoints doesn't
        intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
        helper function.
    */

    // WRITE YOUR CODE HERE
    for (int i = 0; i < terrain->get_map_height(); ++i)
    {
        for (int j = 0; j < terrain->get_map_width(); ++j)
        {
			if (!terrain->is_valid_grid_position({ i,j }) || terrain->is_wall({ i,j }))
				continue;

			if (is_clear_path(row, col, i, j))
			{
				layer.set_value(GridPos{ i,j }, 1.0f);
			}
        	else
			{
                layer.set_value(GridPos{ i,j }, 0.0f);
			}
        }
    }

    for (int i = 0; i < terrain->get_map_height(); ++i)
    {
        for (int j = 0; j < terrain->get_map_width(); ++j)
        {
			if (!terrain->is_valid_grid_position({ i,j }) || terrain->is_wall({ i,j }))
				continue;

            if(layer.get_value(GridPos{i,j}) >= 1.0f)
            {
				//Get surrounding cells
				for (int k = -1; k <= 1; ++k)
				{
					for (int l = -1; l <= 1; ++l)
					{
						if (k == 0 && l == 0)
							continue;

						if (!terrain->is_valid_grid_position({ i + k, j + l }) || terrain->is_wall({ i + k, j + l }))
							continue;

						const bool diagonalNeighbor = (abs(k) == 1 && abs(l) == 1);
                        bool isWalkable = true;

                        if(diagonalNeighbor)
                        {
                            isWalkable = isDiagonalWalkable(i, j, i + k, j + l);
                        }

						if (isWalkable && layer.get_value(GridPos{ i + k, j + l }) == 0.0f)
						{
							layer.set_value(GridPos{ i + k, j + l }, 0.5f);
						}
					}
				}
            }
        }
    }
}

void analyze_agent_vision(MapLayer<float> &layer, const Agent *agent)
{
    /*
        For every cell in the given layer that is visible to the given agent,
        mark it as 1.0, otherwise don't change the cell's current value.

        You must consider the direction the agent is facing.  All of the agent data is
        in three dimensions, but to simplify you should operate in two dimensions, the XZ plane.

        Take the dot product between the view vector and the vector from the agent to the cell,
        both normalized, and compare the cosines directly instead of taking the arccosine to
        avoid introducing floating-point inaccuracy (larger cosine means smaller angle).

        Give the agent a field of view slighter larger than 180 degrees.

        Two cells are visible to each other if a line between their centerpoints doesn't
        intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
        helper function.
    */

    // WRITE YOUR CODE HERE
    for (int i = 0; i < terrain->get_map_height(); ++i)
    {
        for (int j = 0; j < terrain->get_map_width(); ++j)
        {
            if (!terrain->is_valid_grid_position({ i,j }) || terrain->is_wall({ i,j }))
                continue;

            bool isVisible = true;

			Vec2 viewVector = Vec2{ agent->get_forward_vector().x, agent->get_forward_vector().z };
            Vec2 cellPos = Vec2{ terrain->get_world_position(GridPos{ i,j}).x, terrain->get_world_position(GridPos{ i,j }).z };
			Vec2 agentPos = Vec2{ agent->get_position().x, agent->get_position().z };
			Vec2 cellToAgent = agentPos - cellPos;
			cellToAgent.Normalize();
			GridPos agentGridPos = terrain->get_grid_position(agent->get_position());

			float dotProduct = viewVector.Dot(cellToAgent);
			float cosAngle = dotProduct / (viewVector.Length() * cellToAgent.Length());

            //Change Visibility to false if its 180 degree behind the agent
            if (cosAngle >= 0) {
                isVisible = false;
            }

            if (isVisible && is_clear_path(agentGridPos.row, agentGridPos.col, i, j))
            {
                layer.set_value(GridPos{ i,j }, 1.0f);
            }
        }
    }
}

void propagate_solo_occupancy(MapLayer<float> &layer, float decay, float growth)
{
    /*
        For every cell in the given layer:

            1) Get the value of each neighbor and apply decay factor
            2) Keep the highest value from step 1
            3) Linearly interpolate from the cell's current value to the value from step 2
               with the growing factor as a coefficient.  Make use of the lerp helper function.
            4) Store the value from step 3 in a temporary layer.
               A float[40][40] will suffice, no need to dynamically allocate or make a new MapLayer.

        After every cell has been processed into the temporary layer, write the temporary layer into
        the given layer;
    */
    
    // WRITE YOUR CODE HERE

    std::array<std::array<float, 40>, 40> tempLayer;

    for (int i = 0; i < terrain->get_map_height(); ++i)
    {
        for (int j = 0; j < terrain->get_map_width(); ++j)
        {
            float currentValue = layer.get_value(i, j);
            float highestValue = ApplyDecayFromNeighbors(i, j, decay, layer);
            float lerpResult = lerp(currentValue, highestValue, growth);
            tempLayer[i][j] = lerpResult;
        }
    }

    for (int i = 0; i < terrain->get_map_height(); ++i)
    {
        for (int j = 0; j < terrain->get_map_width(); ++j)
        {
            layer.set_value(i, j, tempLayer[i][j]);
        }
    }
}

void propagate_dual_occupancy(MapLayer<float> &layer, float decay, float growth)
{
    /*
        Similar to the solo version, but the values range from -1.0 to 1.0, instead of 0.0 to 1.0

        For every cell in the given layer:

        1) Get the value of each neighbor and apply decay factor
        2) Keep the highest ABSOLUTE value from step 1
        3) Linearly interpolate from the cell's current value to the value from step 2
           with the growing factor as a coefficient.  Make use of the lerp helper function.
        4) Store the value from step 3 in a temporary layer.
           A float[40][40] will suffice, no need to dynamically allocate or make a new MapLayer.

        After every cell has been processed into the temporary layer, write the temporary layer into
        the given layer;
    */

    // WRITE YOUR CODE HERE
}

void normalize_solo_occupancy(MapLayer<float> &layer)
{
    /*
        Determine the maximum value in the given layer, and then divide the value
        for every cell in the layer by that amount.  This will keep the values in the
        range of [0, 1].  Negative values should be left unmodified.
    */

    // WRITE YOUR CODE HERE
}

void normalize_dual_occupancy(MapLayer<float> &layer)
{
    /*
        Similar to the solo version, but you need to track greatest positive value AND 
        the least (furthest from 0) negative value.

        For every cell in the given layer, if the value is currently positive divide it by the
        greatest positive value, or if the value is negative divide it by -1.0 * the least negative value
        (so that it remains a negative number).  This will keep the values in the range of [-1, 1].
    */

    // WRITE YOUR CODE HERE
}

void enemy_field_of_view(MapLayer<float> &layer, float fovAngle, float closeDistance, float occupancyValue, AStarAgent *enemy)
{
    /*
        First, clear out the old values in the map layer by setting any negative value to 0.
        Then, for every cell in the layer that is within the field of view cone, from the
        enemy agent, mark it with the occupancy value.  Take the dot product between the view
        vector and the vector from the agent to the cell, both normalized, and compare the
        cosines directly instead of taking the arccosine to avoid introducing floating-point
        inaccuracy (larger cosine means smaller angle).

        If the tile is close enough to the enemy (less than closeDistance),
        you only check if it's visible to enemy.  Make use of the is_clear_path
        helper function.  Otherwise, you must consider the direction the enemy is facing too.
        This creates a radius around the enemy that the player can be detected within, as well
        as a fov cone.
    */

    // WRITE YOUR CODE HERE
}

bool enemy_find_player(MapLayer<float> &layer, AStarAgent *enemy, Agent *player)
{
    /*
        Check if the player's current tile has a negative value, ie in the fov cone
        or within a detection radius.
    */

    const auto &playerWorldPos = player->get_position();

    const auto playerGridPos = terrain->get_grid_position(playerWorldPos);

    // verify a valid position was returned
    if (terrain->is_valid_grid_position(playerGridPos) == true)
    {
        if (layer.get_value(playerGridPos) < 0.0f)
        {
            return true;
        }
    }

    // player isn't in the detection radius or fov cone, OR somehow off the map
    return false;
}

bool enemy_seek_player(MapLayer<float> &layer, AStarAgent *enemy)
{
    /*
        Attempt to find a cell with the highest nonzero value (normalization may
        not produce exactly 1.0 due to floating point error), and then set it as
        the new target, using enemy->path_to.

        If there are multiple cells with the same highest value, then pick the
        cell closest to the enemy.

        Return whether a target cell was found.
    */

    // WRITE YOUR CODE HERE

    return false; // REPLACE THIS
}
