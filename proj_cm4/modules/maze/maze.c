#include "maze.h"
#include <string.h>
#include "cy_pdl.h"
#include "head.h"
#include "distance.h"
#include "debug_uart.h"
#include "car.h"

#define DIR_COUNT      (4)
#define HEAD_LEFT_DEG  (90)
#define HEAD_RIGHT_DEG (-90)

typedef enum { EDGE_UNKNOWN = 0, EDGE_WALL = 1, EDGE_OPEN = 2 } Edge;

typedef struct {
    uint8_t edges;
    uint8_t marks;
    uint8_t visited;
} Cell;

static Cell maze[MAZE_SIZE][MAZE_SIZE];
static uint8_t pos_x = MAZE_START_X;
static uint8_t pos_y = MAZE_START_Y;
static Maze_Dir heading = MAZE_NORTH;
static bool arrived_in_visited_cell = false;

static uint8_t min_x, max_x, min_y, max_y;

///////////////////// HELPERS /////////////////////////////////////////////////
static Maze_Dir opposite(Maze_Dir dir) {
    return (Maze_Dir) (((int) dir + 2) % DIR_COUNT);
}

static Maze_Dir turnRight(Maze_Dir dir) {
    return (Maze_Dir) (((int) dir + 1) % DIR_COUNT);
}

static Maze_Dir turnLeft(Maze_Dir dir) {
    return (Maze_Dir) (((int) dir + DIR_COUNT - 1) % DIR_COUNT);
}

static bool neighbor(uint8_t x, uint8_t y, Maze_Dir dir, uint8_t* nx, uint8_t* ny) {
    int px = x;
    int py = y;
    switch (dir) {
        case MAZE_NORTH: --py; break;
        case MAZE_EAST: ++px; break;
        case MAZE_SOUTH: ++py; break;
        case MAZE_WEST: --px; break;
        default: return false;
    }
    if (px < 0 || py < 0 || px >= (int) MAZE_SIZE || py >= (int) MAZE_SIZE) {
        return false;
    }
    *nx = (uint8_t) px;
    *ny = (uint8_t) py;
    return true;
}

static Edge getEdge(uint8_t x, uint8_t y, Maze_Dir dir) {
    return (Edge) ((maze[y][x].edges >> (2 * (int) dir)) & 3u);
}

static void putEdge(uint8_t x, uint8_t y, Maze_Dir dir, Edge edge) {
    uint8_t shift = (uint8_t) (2 * (int) dir);
    maze[y][x].edges = (uint8_t) ((maze[y][x].edges & ~(3u << shift)) | ((uint8_t) edge << shift));
}

static void setEdge(uint8_t x, uint8_t y, Maze_Dir dir, Edge edge) {
    uint8_t nx, ny;
    putEdge(x, y, dir, edge);
    if (neighbor(x, y, dir, &nx, &ny)) {
        putEdge(nx, ny, opposite(dir), edge);
    }
}

static uint8_t getMarks(uint8_t x, uint8_t y, Maze_Dir dir) {
    return (uint8_t) ((maze[y][x].marks >> (2 * (int) dir)) & 3u);
}

static void putMarks(uint8_t x, uint8_t y, Maze_Dir dir, uint8_t marks) {
    uint8_t shift = (uint8_t) (2 * (int) dir);
    maze[y][x].marks = (uint8_t) ((maze[y][x].marks & ~(3u << shift)) | ((marks & 3u) << shift));
}

static void setMarks(uint8_t x, uint8_t y, Maze_Dir dir, uint8_t marks) {
    uint8_t nx, ny;
    putMarks(x, y, dir, marks);
    if (neighbor(x, y, dir, &nx, &ny)) {
        putMarks(nx, ny, opposite(dir), marks);
    }
}

static bool canGo(Maze_Dir dir) {
    uint8_t nx, ny;
    return getEdge(pos_x, pos_y, dir) == EDGE_OPEN && neighbor(pos_x, pos_y, dir, &nx, &ny);
}

///////////////////// MAP /////////////////////////////////////////////////////
void Maze_Init(void) {
    memset(maze, 0, sizeof(maze));
    pos_x = MAZE_START_X;
    pos_y = MAZE_START_Y;
    heading = MAZE_NORTH;
    arrived_in_visited_cell = false;
    min_x = max_x = MAZE_START_X;
    min_y = max_y = MAZE_START_Y;

    setEdge(pos_x, pos_y, MAZE_SOUTH, EDGE_OPEN);
    setMarks(pos_x, pos_y, MAZE_SOUTH, MAZE_MAX_MARKS);

    DBG_PRINTF("@R %u %u\n", (unsigned) pos_x, (unsigned) pos_y);
}

void Maze_SetWalls(bool wall_front, bool wall_left, bool wall_right) {
    setEdge(pos_x, pos_y, heading, wall_front ? EDGE_WALL : EDGE_OPEN);
    setEdge(pos_x, pos_y, turnLeft(heading), wall_left ? EDGE_WALL : EDGE_OPEN);
    setEdge(pos_x, pos_y, turnRight(heading), wall_right ? EDGE_WALL : EDGE_OPEN);
    maze[pos_y][pos_x].visited = 1;

    min_x = (pos_x < min_x) ? pos_x : min_x;
    max_x = (pos_x > max_x) ? pos_x : max_x;
    min_y = (pos_y < min_y) ? pos_y : min_y;
    max_y = (pos_y > max_y) ? pos_y : max_y;

    DBG_PRINTF("@C %u %u %u\n", (unsigned) pos_x, (unsigned) pos_y, (unsigned) maze[pos_y][pos_x].edges);
}

static bool wallAt(int8_t pan_deg, uint16_t threshold_mm) {
    uint16_t mm;
    if (Head_GetPan() != pan_deg) {
        Head_Look(pan_deg, 0);
        Car_Wait(MAZE_HEAD_SETTLE_MS);
    }
    Distance_Start();
    Car_Wait(MAZE_MEASURE_MS);
    if (!Distance_GetMm(&mm)) {
        return false;
    }
    return mm < threshold_mm;
}

static bool wallInDir(Maze_Dir dir, int8_t pan_deg, uint16_t threshold_mm) {
    Edge known = getEdge(pos_x, pos_y, dir);
    if (known != EDGE_UNKNOWN) {
        return known == EDGE_WALL;
    }
    return wallAt(pan_deg, threshold_mm);
}

void Maze_ScanCell(void) {
    if (maze[pos_y][pos_x].visited) {
        return;
    }
    bool left = wallInDir(turnLeft(heading), HEAD_LEFT_DEG, MAZE_SIDE_WALL_MM);
    bool right = wallInDir(turnRight(heading), HEAD_RIGHT_DEG, MAZE_SIDE_WALL_MM);
    bool front = wallInDir(heading, 0, MAZE_FRONT_WALL_MM);
    Maze_SetWalls(front, left, right);
}

///////////////////// TREMAUX /////////////////////////////////////////////////
static const int8_t PREFERENCE[DIR_COUNT] = {0, -1, 1, 2};

static Maze_Dir relativeDir(int8_t quarter_turns_right) {
    return (Maze_Dir) ((((int) heading + quarter_turns_right) % DIR_COUNT + DIR_COUNT) % DIR_COUNT);
}

Maze_Dir Maze_ChooseNext(void) {
    Maze_Dir back = opposite(heading);

    if (arrived_in_visited_cell && canGo(back) && getMarks(pos_x, pos_y, back) == 1) {
        return back;
    }

    for (uint8_t wanted = 0; wanted < MAZE_MAX_MARKS; ++wanted) {
        for (uint8_t i = 0; i < DIR_COUNT; ++i) {
            Maze_Dir dir = relativeDir(PREFERENCE[i]);
            if (canGo(dir) && getMarks(pos_x, pos_y, dir) == wanted) {
                return dir;
            }
        }
    }
    return MAZE_DIR_NONE;
}

int16_t Maze_TurnDegrees(Maze_Dir dir) {
    switch (((int) dir - (int) heading + DIR_COUNT) % DIR_COUNT) {
        case 1: return -90;
        case 2: return 180;
        case 3: return 90;
        default: return 0;
    }
}

bool Maze_Advance(Maze_Dir dir) {
    uint8_t nx, ny;
    if (dir >= MAZE_DIR_NONE || !canGo(dir) || !neighbor(pos_x, pos_y, dir, &nx, &ny)) {
        return false;
    }

    uint8_t marks = getMarks(pos_x, pos_y, dir);
    if (marks < MAZE_MAX_MARKS) {
        marks = (uint8_t) (marks + 1u);
        setMarks(pos_x, pos_y, dir, marks);
    }
    DBG_PRINTF("@M %u %u %u %u\n", (unsigned) pos_x, (unsigned) pos_y, (unsigned) dir, (unsigned) marks);
    arrived_in_visited_cell = maze[ny][nx].visited != 0;
    pos_x = nx;
    pos_y = ny;
    heading = dir;
    return true;
}

///////////////////// STATE ///////////////////////////////////////////////////
uint8_t Maze_GetX(void) {
    return pos_x;
}

uint8_t Maze_GetY(void) {
    return pos_y;
}

Maze_Dir Maze_GetHeading(void) {
    return heading;
}

///////////////////// DEBUG ///////////////////////////////////////////////////
static char line[4 * MAZE_SIZE + 4];

static char verticalEdgeChar(uint8_t x, uint8_t y, Maze_Dir dir) {
    switch (getEdge(x, y, dir)) {
        case EDGE_WALL: return '|';
        case EDGE_OPEN: {
            uint8_t marks = getMarks(x, y, dir);
            return marks == 0 ? ' ' : (char) ('0' + marks);
        }
        default: return '?';
    }
}

static void putHorizontalEdge(char* out, uint8_t x, uint8_t y, Maze_Dir dir) {
    switch (getEdge(x, y, dir)) {
        case EDGE_WALL: out[0] = out[1] = out[2] = '-'; break;
        case EDGE_OPEN: {
            uint8_t marks = getMarks(x, y, dir);
            out[0] = out[2] = (marks == 0) ? ' ' : '-';
            out[1] = (marks == 0) ? ' ' : (char) ('0' + marks);
            if (x == MAZE_START_X && y == MAZE_START_Y && dir == MAZE_SOUTH) {
                out[1] = 'X';
            }
            break;
        }
        default:
            out[0] = out[2] = ' ';
            out[1] = '?';
            break;
    }
}

static void printHorizontalLine(uint8_t y, Maze_Dir dir) {
    char* p = line;
    *p++ = '+';
    for (uint8_t x = min_x; x <= max_x; ++x) {
        putHorizontalEdge(p, x, y, dir);
        p += 3;
        *p++ = '+';
    }
    *p = '\0';
    DBG_PRINTF_LOCAL("%s\n", line);
}

static void printCellLine(uint8_t y) {
    char* p = line;
    *p++ = verticalEdgeChar(min_x, y, MAZE_WEST);
    for (uint8_t x = min_x; x <= max_x; ++x) {
        char content = ' ';
        if (x == pos_x && y == pos_y) {
            content = 'R';
        } else if (x == MAZE_START_X && y == MAZE_START_Y) {
            content = 'S';
        } else if (maze[y][x].visited) {
            content = '.';
        }
        *p++ = ' ';
        *p++ = content;
        *p++ = ' ';
        *p++ = verticalEdgeChar(x, y, MAZE_EAST);
    }
    *p = '\0';
    DBG_PRINTF_LOCAL("%s\n", line);
}

void Maze_Print(void) {
    for (uint8_t y = min_y; y <= max_y; ++y) {
        printHorizontalLine(y, MAZE_NORTH);
        printCellLine(y);
    }
    printHorizontalLine(max_y, MAZE_SOUTH);
    DBG_PRINTF_LOCAL("pos (%u,%u) heading %d\n", (unsigned) pos_x, (unsigned) pos_y, (int) heading);
}
