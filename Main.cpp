#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <algorithm>
#include "API.h"

using namespace std;

// --- CONFIGURATION ---
const bool START_FROM_RIGHT = false;
const int MAZE_SIZE = 16;

// Direction constants
const int NORTH = 0;
const int EAST = 1;
const int SOUTH = 2;
const int WEST = 3;

// Global variables
int x = START_FROM_RIGHT ? (MAZE_SIZE - 1) : 0;
int y = 0;
int orient = NORTH;

bool walls[MAZE_SIZE][MAZE_SIZE][4];
int flood[MAZE_SIZE][MAZE_SIZE];
bool visited[MAZE_SIZE][MAZE_SIZE]; // Tracks cells we have actually entered
bool controller = true;

struct Coord
{
    int x;
    int y;
};

vector<Coord> mazepath;

void log(string text)
{
    cerr << text << endl;
}

void initMaze()
{
    for (int i = 0; i < MAZE_SIZE; i++)
    {
        for (int j = 0; j < MAZE_SIZE; j++)
        {
            visited[i][j] = false;
            walls[i][j][NORTH] = false;
            walls[i][j][EAST] = false;
            walls[i][j][SOUTH] = false;
            walls[i][j][WEST] = false;

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

void updateWalls()
{
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
            walls[nx][ny][(orient + 2) % 4] = true;
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

// Standard Flood Fill (Exploration Mode)
// Assumes unknown areas are open (optimistic)
void floodFill()
{
    for (int i = 0; i < MAZE_SIZE; i++)
    {
        for (int j = 0; j < MAZE_SIZE; j++)
        {
            flood[i][j] = 999;
        }
    }
    queue<Coord> q;
    flood[7][7] = 0;
    q.push({7, 7});
    flood[7][8] = 0;
    q.push({7, 8});
    flood[8][7] = 0;
    q.push({8, 7});
    flood[8][8] = 0;
    q.push({8, 8});

    while (!q.empty())
    {
        Coord c = q.front();
        q.pop();
        int dist = flood[c.x][c.y];

        if (!walls[c.x][c.y][NORTH] && c.y < MAZE_SIZE - 1)
        {
            if (flood[c.x][c.y + 1] == 999)
            {
                flood[c.x][c.y + 1] = dist + 1;
                q.push({c.x, c.y + 1});
            }
        }
        if (!walls[c.x][c.y][EAST] && c.x < MAZE_SIZE - 1)
        {
            if (flood[c.x + 1][c.y] == 999)
            {
                flood[c.x + 1][c.y] = dist + 1;
                q.push({c.x + 1, c.y});
            }
        }
        if (!walls[c.x][c.y][SOUTH] && c.y > 0)
        {
            if (flood[c.x][c.y - 1] == 999)
            {
                flood[c.x][c.y - 1] = dist + 1;
                q.push({c.x, c.y - 1});
            }
        }
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

// VISITED Flood Fill (Path Calculation Mode)
// Only flows through cells we have actually visited.
// This ensures the "slope" of values leads us back home safely.
void floodFillVisited()
{
    for (int i = 0; i < MAZE_SIZE; i++)
    {
        for (int j = 0; j < MAZE_SIZE; j++)
        {
            flood[i][j] = 999;
        }
    }
    queue<Coord> q;
    flood[7][7] = 0;
    q.push({7, 7});
    flood[7][8] = 0;
    q.push({7, 8});
    flood[8][7] = 0;
    q.push({8, 7});
    flood[8][8] = 0;
    q.push({8, 8});

    while (!q.empty())
    {
        Coord c = q.front();
        q.pop();
        int dist = flood[c.x][c.y];

        // Check North (Must be No Wall AND Visited)
        if (!walls[c.x][c.y][NORTH] && c.y < MAZE_SIZE - 1)
        {
            if (flood[c.x][c.y + 1] == 999 && visited[c.x][c.y + 1])
            {
                flood[c.x][c.y + 1] = dist + 1;
                q.push({c.x, c.y + 1});
            }
        }
        // Check East
        if (!walls[c.x][c.y][EAST] && c.x < MAZE_SIZE - 1)
        {
            if (flood[c.x + 1][c.y] == 999 && visited[c.x + 1][c.y])
            {
                flood[c.x + 1][c.y] = dist + 1;
                q.push({c.x + 1, c.y});
            }
        }
        // Check South
        if (!walls[c.x][c.y][SOUTH] && c.y > 0)
        {
            if (flood[c.x][c.y - 1] == 999 && visited[c.x][c.y - 1])
            {
                flood[c.x][c.y - 1] = dist + 1;
                q.push({c.x, c.y - 1});
            }
        }
        // Check West
        if (!walls[c.x][c.y][WEST] && c.x > 0)
        {
            if (flood[c.x - 1][c.y] == 999 && visited[c.x - 1][c.y])
            {
                flood[c.x - 1][c.y] = dist + 1;
                q.push({c.x - 1, c.y});
            }
        }
    }
}

void moveNext()
{
    int minVal = 999;
    int bestDir = -1;

    if (!walls[x][y][NORTH] && y < MAZE_SIZE - 1 && flood[x][y + 1] < minVal)
    {
        minVal = flood[x][y + 1];
        bestDir = NORTH;
    }
    if (!walls[x][y][EAST] && x < MAZE_SIZE - 1 && flood[x + 1][y] < minVal)
    {
        minVal = flood[x + 1][y];
        bestDir = EAST;
    }
    if (!walls[x][y][SOUTH] && y > 0 && flood[x][y - 1] < minVal)
    {
        minVal = flood[x][y - 1];
        bestDir = SOUTH;
    }
    if (!walls[x][y][WEST] && x > 0 && flood[x - 1][y] < minVal)
    {
        minVal = flood[x - 1][y];
        bestDir = WEST;
    }

    if (bestDir != -1)
    {
        int diff = bestDir - orient;
        if (diff == 0)
            API::moveForward();
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

        if (orient == NORTH)
            y++;
        else if (orient == EAST)
            x++;
        else if (orient == SOUTH)
            y--;
        else if (orient == WEST)
            x--;
    }
    visited[x][y] = true;
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

// Calculate Optimal Path based on Flood Values (Gradient Descent)
vector<Coord> getOptimalPath(int startX, int startY)
{
    vector<Coord> path;
    int currX = startX;
    int currY = startY;

    path.push_back({currX, currY});

    // Loop until we reach the goal (dist 0)
    // Safety break: max 256 steps to prevent infinite loop if bugs
    int steps = 0;
    while (flood[currX][currY] != 0 && steps < 300)
    {
        int minVal = flood[currX][currY];
        int nextX = -1, nextY = -1;

        // Check neighbors for strictly lower value
        // Note: We don't need to check "visited" here because floodFillVisited()
        // already ensured only visited cells have valid numbers.

        if (!walls[currX][currY][NORTH] && currY < MAZE_SIZE - 1 && flood[currX][currY + 1] < minVal)
        {
            minVal = flood[currX][currY + 1];
            nextX = currX;
            nextY = currY + 1;
        }
        if (!walls[currX][currY][EAST] && currX < MAZE_SIZE - 1 && flood[currX + 1][currY] < minVal)
        {
            minVal = flood[currX + 1][currY];
            nextX = currX + 1;
            nextY = currY;
        }
        if (!walls[currX][currY][SOUTH] && currY > 0 && flood[currX][currY - 1] < minVal)
        {
            minVal = flood[currX][currY - 1];
            nextX = currX;
            nextY = currY - 1;
        }
        if (!walls[currX][currY][WEST] && currX > 0 && flood[currX - 1][currY] < minVal)
        {
            minVal = flood[currX - 1][currY];
            nextX = currX - 1;
            nextY = currY;
        }

        if (nextX != -1)
        {
            currX = nextX;
            currY = nextY;
            path.push_back({currX, currY});
        }
        else
        {
            log("Error: Path broken at " + to_string(currX) + "," + to_string(currY));
            break;
        }
        steps++;
    }
    return path;
}

// Can run the path forwards (Start->Goal) or backwards (Goal->Start)
void fastrun(vector<Coord> path, bool reversePath)
{
    vector<Coord> runPath = path;
    if (reversePath)
    {
        reverse(runPath.begin(), runPath.end());
    }

    // Start loop from 1 because index 0 is our current position
    for (size_t i = 1; i < runPath.size(); i++)
    {
        Coord target = runPath[i];

        if (target.x == x && target.y == y)
            continue;

        // Log less frequently to avoid spam, or log every step if needed
        // log("Move: " + to_string(target.x) + "," + to_string(target.y));

        int desiredDir;
        if (target.x > x)
            desiredDir = EAST;
        else if (target.x < x)
            desiredDir = WEST;
        else if (target.y > y)
            desiredDir = NORTH;
        else
            desiredDir = SOUTH;

        int diff = desiredDir - orient;
        if (diff == 0)
        { /* No turn */
        }
        else if (diff == 1 || diff == -3)
        {
            API::turnRight();
            orient = (orient + 1) % 4;
        }
        else if (diff == -1 || diff == 3)
        {
            API::turnLeft();
            orient = (orient + 3) % 4;
        }
        else
        {
            API::turnRight();
            API::turnRight();
            orient = (orient + 2) % 4;
        }

        API::moveForward();
        x = target.x;
        y = target.y;
    }
}

int main(int argc, char *argv[])
{
    log("Running...");
    API::setColor(x, y, 'G');
    API::setText(x, y, "Start");

    initMaze();

    // --- 1. SEARCH PHASE ---
    while (controller)
    {
        visited[x][y] = true;
        API::setColor(x, y, 'G');

        updateWalls();
        floodFill(); // Normal optimistic flood fill
        showFloodValues();

        if ((x == 7 || x == 8) && (y == 7 || y == 8))
        {
            log("Goal Reached!");
            visited[x][y] = true; // Mark center as visited
            controller = false;
        }
        else
        {
            moveNext();
        }
    }

    // --- 2. RE-CALCULATE FOR VISITED PATH ---
    log("Recalculating flood for visited cells only...");
    floodFillVisited();
    showFloodValues(); // Update UI to show the 'safe' numbers

    // --- 3. CALCULATE OPTIMAL PATH ---
    int startX = START_FROM_RIGHT ? (MAZE_SIZE - 1) : 0;
    int startY = 0;
    mazepath = getOptimalPath(startX, startY);

    // Print Path to Console
    log("--- OPTIMAL PATH (" + to_string(mazepath.size()) + " steps) ---");
    for (size_t i = 0; i < mazepath.size(); i++)
    {
        string s = to_string(i) + ": (" + to_string(mazepath[i].x) + "," + to_string(mazepath[i].y) + ")";
        log(s);
        API::setColor(mazepath[i].x, mazepath[i].y, 'B'); // Blue path
    }
    log("--------------------------------");

    // --- 4. RETURN TO START ---
    log("Returning to Start...");
    fastrun(mazepath, true); // Reverse = true

    // --- 5. ALIGNMENT ---
    // Ensure we are facing NORTH (or valid start dir) before fast run
    while (orient != NORTH)
    {
        API::turnRight();
        orient = (orient + 1) % 4;
    }

    // --- 6. FAST RUN ---
    log("Starting Fast Run...");
    fastrun(mazepath, false); // Reverse = false
}