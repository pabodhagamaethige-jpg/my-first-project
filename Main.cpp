#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <algorithm>
#include "API.h"

using namespace std;

// --- CONFIGURATION ---
// Set to 'true' to start at Bottom-Right (15, 0)
// Set to 'false' to start at Bottom-Left (0, 0)
const bool START_FROM_RIGHT = false;

// Micromouse maze is usually 16x16
const int MAZE_SIZE = 16;

// Direction constants
const int NORTH = 0;
const int EAST = 1;
const int SOUTH = 2;
const int WEST = 3;

// Global variables to store the state
// IF STARTING RIGHT: x is 15. IF STARTING LEFT: x is 0.
int x = START_FROM_RIGHT ? (MAZE_SIZE - 1) : 0;
int y = 0;

// Note: Ensure your simulator spawns the mouse facing NORTH.
int orient = NORTH; // 0: North, 1: East, 2: South, 3: West

// 2D array to store wall data.
// walls[x][y][direction] = true if there is a wall
bool walls[MAZE_SIZE][MAZE_SIZE][4];

bool controller = true;

// 2D array to store the "flood" values (distances)
int flood[MAZE_SIZE][MAZE_SIZE];

struct Coord
{
    int x;
    int y;
};

// Helper function to log text to the simulator console
void log(string text)
{
    cerr << text << endl;
}

// Initialize the maze memory
void initMaze()
{
    for (int i = 0; i < MAZE_SIZE; i++)
    {
        for (int j = 0; j < MAZE_SIZE; j++)
        {
            // Assume no walls exist initially
            walls[i][j][NORTH] = false;
            walls[i][j][EAST] = false;
            walls[i][j][SOUTH] = false;
            walls[i][j][WEST] = false;

            // Set bounds (outer walls)
            if (j == MAZE_SIZE - 1)
                walls[i][j][NORTH] = true;
            if (i == MAZE_SIZE - 1)
                walls[i][j][EAST] = true;
            if (j == 0)
                walls[i][j][SOUTH] = true;
            if (i == 0)
                walls[i][j][WEST] = true;
        }
    }
}

// Update the global 'walls' array based on what the sensors see
void updateWalls()
{
    // Current orientation determines which global direction "Front", "Left", "Right" map to
    if (API::wallFront())
    {
        walls[x][y][orient] = true;
        int nx = x, ny = y;
        if (orient == NORTH)
            ny++;
        else if (orient == EAST)
            nx++;
        else if (orient == SOUTH)
            ny--;
        else if (orient == WEST)
            nx--;

        if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE)
        {
            walls[nx][ny][(orient + 2) % 4] = true;
        }
    }

    if (API::wallRight())
    {
        int dir = (orient + 1) % 4;
        walls[x][y][dir] = true;
        int nx = x, ny = y;
        if (dir == NORTH)
            ny++;
        else if (dir == EAST)
            nx++;
        else if (dir == SOUTH)
            ny--;
        else if (dir == WEST)
            nx--;
        if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE)
            walls[nx][ny][(dir + 2) % 4] = true;
    }

    if (API::wallLeft())
    {
        int dir = (orient + 3) % 4;
        walls[x][y][dir] = true;
        int nx = x, ny = y;
        if (dir == NORTH)
            ny++;
        else if (dir == EAST)
            nx++;
        else if (dir == SOUTH)
            ny--;
        else if (dir == WEST)
            nx--;
        if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE)
            walls[nx][ny][(dir + 2) % 4] = true;
    }
}

// THE FLOOD FILL ALGORITHM
void floodFill()
{
    // 1. Reset all distances to a high number (infinity)
    for (int i = 0; i < MAZE_SIZE; i++)
    {
        for (int j = 0; j < MAZE_SIZE; j++)
        {
            flood[i][j] = 999;
        }
    }

    queue<Coord> q;

    // 2. Set the goal (Center of 16x16 maze)
    flood[7][7] = 0;
    q.push({7, 7});
    flood[7][8] = 0;
    q.push({7, 8});
    flood[8][7] = 0;
    q.push({8, 7});
    flood[8][8] = 0;
    q.push({8, 8});

    // 3. Process the queue
    while (!q.empty())
    {
        Coord c = q.front();
        q.pop();

        int dist = flood[c.x][c.y];

        // Check North Neighbor
        if (!walls[c.x][c.y][NORTH] && c.y < MAZE_SIZE - 1)
        {
            if (flood[c.x][c.y + 1] == 999)
            {
                flood[c.x][c.y + 1] = dist + 1;
                q.push({c.x, c.y + 1});
            }
        }
        // Check East Neighbor
        if (!walls[c.x][c.y][EAST] && c.x < MAZE_SIZE - 1)
        {
            if (flood[c.x + 1][c.y] == 999)
            {
                flood[c.x + 1][c.y] = dist + 1;
                q.push({c.x + 1, c.y});
            }
        }
        // Check South Neighbor
        if (!walls[c.x][c.y][SOUTH] && c.y > 0)
        {
            if (flood[c.x][c.y - 1] == 999)
            {
                flood[c.x][c.y - 1] = dist + 1;
                q.push({c.x, c.y - 1});
            }
        }
        // Check West Neighbor
        if (!walls[c.x][c.y][WEST] && c.x > 0)
        {
            if (flood[c.x - 1][c.y] == 999)
            {
                flood[c.x - 1][c.y] = dist + 1;
                q.push({c.x - 1, c.y});
            }
        }
    }
}

void moveNext()
{
    int currentDist = flood[x][y];
    int bestDir = -1;
    int minVal = 999;

    // Check North
    if (!walls[x][y][NORTH] && y < MAZE_SIZE - 1)
    {
        if (flood[x][y + 1] < minVal)
        {
            minVal = flood[x][y + 1];
            bestDir = NORTH;
        }
    }
    // Check East
    if (!walls[x][y][EAST] && x < MAZE_SIZE - 1)
    {
        if (flood[x + 1][y] < minVal)
        {
            minVal = flood[x + 1][y];
            bestDir = EAST;
        }
    }
    // Check South
    if (!walls[x][y][SOUTH] && y > 0)
    {
        if (flood[x][y - 1] < minVal)
        {
            minVal = flood[x][y - 1];
            bestDir = SOUTH;
        }
    }
    // Check West
    if (!walls[x][y][WEST] && x > 0)
    {
        if (flood[x - 1][y] < minVal)
        {
            minVal = flood[x - 1][y];
            bestDir = WEST;
        }
    }

    // Move logic
    if (bestDir != -1)
    {
        int diff = bestDir - orient;
        if (diff == 0)
        {
            API::moveForward();
        }
        else if (diff == 1 || diff == -3)
        {
            API::turnRight();
            API::moveForward();
            orient = (orient + 1) % 4;
        }
        else if (diff == -1 || diff == 3)
        {
            API::turnLeft();
            API::moveForward();
            orient = (orient + 3) % 4;
        }
        else
        {
            API::turnRight();
            API::turnRight();
            API::moveForward();
            orient = (orient + 2) % 4;
        }

        // Update coordinate
        if (orient == NORTH)
            y++;
        else if (orient == EAST)
            x++;
        else if (orient == SOUTH)
            y--;
        else if (orient == WEST)
            x--;
    }
}

void showFloodValues()
{
    for (int i = 0; i < MAZE_SIZE; i++)
    {
        for (int j = 0; j < MAZE_SIZE; j++)
        {
            API::setText(i, j, to_string(flood[i][j]));
        }
    }
}

void save_path()
{
    int currentDist = flood[x][y];
    int bestDir = -1;
    int minVal = 999;

    // Check North
    if (!walls[x][y][NORTH] && y < MAZE_SIZE - 1)
    {
        if (flood[x][y + 1] < minVal)
        {
            minVal = flood[x][y + 1];
            bestDir = NORTH;
        }
    }
    // Check East
    if (!walls[x][y][EAST] && x < MAZE_SIZE - 1)
    {
        if (flood[x + 1][y] < minVal)
        {
            minVal = flood[x + 1][y];
            bestDir = EAST;
        }
    }
    // Check South
    if (!walls[x][y][SOUTH] && y > 0)
    {
        if (flood[x][y - 1] < minVal)
        {
            minVal = flood[x][y - 1];
            bestDir = SOUTH;
        }
    }
    // Check West
    if (!walls[x][y][WEST] && x > 0)
    {
        if (flood[x - 1][y] < minVal)
        {
            minVal = flood[x - 1][y];
            bestDir = WEST;
        }
    }

    // Move logic
    if (bestDir != -1)
    {
        int diff = bestDir - orient;
        if (diff == 0)
        {
        }
        else if (diff == 1 || diff == -3)
        {
            orient = (orient + 1) % 4;
        }
        else if (diff == -1 || diff == 3)
        {
            orient = (orient + 3) % 4;
        }
        else
        {
            orient = (orient + 2) % 4;
        }

        // Update coordinate
        if (orient == NORTH)
            y++;
        else if (orient == EAST)
            x++;
        else if (orient == SOUTH)
            y--;
        else if (orient == WEST)
            x--;
    }
}

int main(int argc, char *argv[])
{
    log("Running...");

    // Set the Start color text based on current position
    API::setColor(x, y, 'G');
    API::setText(x, y, "Start");

    initMaze();

    while (controller)
    {
        updateWalls();
        floodFill();
        showFloodValues();

        // Stop if we reach the center (works from any starting side)
        if (!((x == 7 || x == 8) && (y == 7 || y == 8)))
        {
            moveNext();
        }
        else
        {
            // Optional: Victory spin or stop
            log("Goal Reached!");
            controller = false;
        }
    }

    std::queue<Coord> main_path;
    x = START_FROM_RIGHT ? (MAZE_SIZE - 1) : 0;
    y = 0;
    orient = NORTH;
    main_path.push({x, y});
    while (!((x == 7 || x == 8) && (y == 7 || y == 8)))
    {
        save_path();
        main_path.push({x, y});
    }

    std::cerr << "Queue: ";
    while (!main_path.empty())
    {
        std::cerr << "(" << main_path.front().x << "," << main_path.front().y << ") " << endl;
        main_path.pop();
    }
    std::cerr << std::endl;
}