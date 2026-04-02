/**
 * @file  maze_solver.c
 * @brief Maze-solving algorithms: right-hand rule and flood-fill.
 *
 * ─────────────────────────────────────────────────────────────────────────
 * Right-Hand Rule  (default, MAZE_ALGO_RIGHT_HAND)
 * ─────────────────────────────────────────────────────────────────────────
 * The car treats walls as a guide: it keeps its right "hand" touching the
 * right wall and navigates accordingly.
 *
 * Decision priority at each cell crossing:
 *   1. Right open  → turn right, then move forward.
 *   2. Front open  → go straight.
 *   3. Left open   → turn left, then move forward.
 *   4. All blocked → U-turn (180°), then move forward.
 *
 * The state machine steps are:
 *   IDLE → (start trigger) → FORWARD/TURN_RIGHT/TURN_LEFT/UTURN → loop
 *
 * ─────────────────────────────────────────────────────────────────────────
 * Flood-Fill  (MAZE_ALGO_FLOOD_FILL)
 * ─────────────────────────────────────────────────────────────────────────
 * A Manhattan-distance map is maintained in s_flood[MAZE_ROWS][MAZE_COLS].
 * Cells are updated with a BFS from the goal. The car navigates toward the
 * neighbour with the smallest flood value that has no wall between it and
 * the current cell.  Wall information is learned on-the-fly.
 *
 * The goal is assumed to be the centre 2×2 cells of the maze
 * (indices [7][7], [7][8], [8][7], [8][8] for a 16×16 maze).
 */

#include <stdio.h>
#include <string.h>

#include "maze_solver.h"
#include "main.h"
#include "motor.h"
#include "ultrasonic.h"
#include "pid.h"

/* ═══════════════════════════════════════════════════════════════════════════
 * Shared state
 * ═══════════════════════════════════════════════════════════════════════════ */
static MazeState_t s_state;
static Heading_t   s_heading;
static uint32_t    s_action_start_ms;   /* timestamp when current action began  */

/* Wall-following PID (right / left sensor setpoint = desired gap in cm)    */
static PID_t       s_wall_pid;
#define WALL_SETPOINT_CM    12.0f   /* desired distance from side wall (cm) */
#define PID_KP               1.8f
#define PID_KI               0.05f
#define PID_KD               0.8f

/* ═══════════════════════════════════════════════════════════════════════════
 * Flood-fill state  (only compiled when MAZE_ALGO_FLOOD_FILL is defined)
 * ═══════════════════════════════════════════════════════════════════════════ */
#ifdef MAZE_ALGO_FLOOD_FILL

/* Bit-packed wall flags for each cell.                                      */
#define WALL_NORTH  0x01U
#define WALL_EAST   0x02U
#define WALL_SOUTH  0x04U
#define WALL_WEST   0x08U
#define WALL_VISITED 0x10U

static uint8_t  s_walls[MAZE_ROWS][MAZE_COLS];
static uint16_t s_flood[MAZE_ROWS][MAZE_COLS];
static int8_t   s_pos_row;
static int8_t   s_pos_col;

/* Goal = centre 2×2 of a 16×16 maze                                        */
#define GOAL_ROW1  (MAZE_ROWS / 2 - 1)
#define GOAL_ROW2  (MAZE_ROWS / 2)
#define GOAL_COL1  (MAZE_COLS / 2 - 1)
#define GOAL_COL2  (MAZE_COLS / 2)

#define FLOOD_INF  0xFFFFU

/* Simple queue for BFS (worst case MAZE_ROWS × MAZE_COLS entries)          */
typedef struct { int8_t row; int8_t col; } Cell_t;
static Cell_t   s_queue[MAZE_ROWS * MAZE_COLS];

/* ── Flood-fill helpers ─────────────────────────────────────────────────── */

static void flood_init(void)
{
    /* Set goal cells to 0, all others to INF                               */
    for (int r = 0; r < MAZE_ROWS; r++)
        for (int c = 0; c < MAZE_COLS; c++)
            s_flood[r][c] = FLOOD_INF;

    s_flood[GOAL_ROW1][GOAL_COL1] = 0;
    s_flood[GOAL_ROW1][GOAL_COL2] = 0;
    s_flood[GOAL_ROW2][GOAL_COL1] = 0;
    s_flood[GOAL_ROW2][GOAL_COL2] = 0;
}

/**
 * @brief BFS update of the flood array from all goal cells outward.
 *        Only propagates through cells that do not have a wall on the
 *        relevant face.
 */
static void flood_update(void)
{
    /* Re-initialise all non-goal cells                                      */
    for (int r = 0; r < MAZE_ROWS; r++)
        for (int c = 0; c < MAZE_COLS; c++)
            s_flood[r][c] = FLOOD_INF;

    s_flood[GOAL_ROW1][GOAL_COL1] = 0;
    s_flood[GOAL_ROW1][GOAL_COL2] = 0;
    s_flood[GOAL_ROW2][GOAL_COL1] = 0;
    s_flood[GOAL_ROW2][GOAL_COL2] = 0;

    uint16_t head = 0, tail = 0;
    /* Seed the queue with goal cells                                        */
    s_queue[tail++] = (Cell_t){ GOAL_ROW1, GOAL_COL1 };
    s_queue[tail++] = (Cell_t){ GOAL_ROW1, GOAL_COL2 };
    s_queue[tail++] = (Cell_t){ GOAL_ROW2, GOAL_COL1 };
    s_queue[tail++] = (Cell_t){ GOAL_ROW2, GOAL_COL2 };

    const int8_t dr[4] = { -1,  0,  1,  0 };  /* N E S W                   */
    const int8_t dc[4] = {  0,  1,  0, -1 };
    const uint8_t wall_flag[4] = { WALL_NORTH, WALL_EAST,
                                   WALL_SOUTH, WALL_WEST };

    while (head != tail)
    {
        Cell_t cur = s_queue[head++];
        uint16_t next_val = s_flood[cur.row][cur.col] + 1U;

        for (uint8_t d = 0; d < 4U; d++)
        {
            int8_t nr = (int8_t)(cur.row + dr[d]);
            int8_t nc = (int8_t)(cur.col + dc[d]);

            if (nr < 0 || nr >= MAZE_ROWS || nc < 0 || nc >= MAZE_COLS) continue;
            if (s_walls[cur.row][cur.col] & wall_flag[d])              continue;
            if (s_flood[nr][nc] <= next_val)                           continue;

            s_flood[nr][nc]  = next_val;
            s_queue[tail++]  = (Cell_t){ nr, nc };
        }
    }
}

/**
 * @brief Learn walls around the current cell from sensor readings and
 *        update the wall map.
 */
static void flood_learn_walls(void)
{
    /* Heading-to-wall-flag mapping for the three sensors                   */
    /* FRONT sensor → wall in current heading direction                     */
    /* LEFT  sensor → wall to the left of heading                          */
    /* RIGHT sensor → wall to the right of heading                         */

    const uint8_t wall_ahead[4] = {
        WALL_NORTH, WALL_EAST, WALL_SOUTH, WALL_WEST
    };
    const uint8_t wall_right[4] = {
        WALL_EAST,  WALL_SOUTH, WALL_WEST,  WALL_NORTH
    };
    const uint8_t wall_left[4] = {
        WALL_WEST,  WALL_NORTH, WALL_EAST,  WALL_SOUTH
    };

    uint8_t *cell = &s_walls[s_pos_row][s_pos_col];
    *cell |= WALL_VISITED;

    if (Ultrasonic_WallFront())  *cell |= wall_ahead[s_heading];
    if (Ultrasonic_WallRight())  *cell |= wall_right[s_heading];
    if (Ultrasonic_WallLeft())   *cell |= wall_left[s_heading];
}

/**
 * @brief Choose the best next move using flood-fill values.
 *        Returns the direction to move (as a Heading_t), or -1 if stuck.
 */
static int8_t flood_best_direction(void)
{
    const int8_t dr[4]        = { -1,  0,  1,  0 };
    const int8_t dc[4]        = {  0,  1,  0, -1 };
    const uint8_t wall_flag[4] = { WALL_NORTH, WALL_EAST,
                                   WALL_SOUTH, WALL_WEST };

    uint16_t best_val = FLOOD_INF;
    int8_t   best_dir = -1;

    for (uint8_t d = 0; d < 4U; d++)
    {
        if (s_walls[s_pos_row][s_pos_col] & wall_flag[d]) continue;

        int8_t nr = (int8_t)(s_pos_row + dr[d]);
        int8_t nc = (int8_t)(s_pos_col + dc[d]);

        if (nr < 0 || nr >= MAZE_ROWS || nc < 0 || nc >= MAZE_COLS) continue;

        if (s_flood[nr][nc] < best_val)
        {
            best_val = s_flood[nr][nc];
            best_dir = (int8_t)d;
        }
    }
    return best_dir;
}

#endif /* MAZE_ALGO_FLOOD_FILL */

/* ═══════════════════════════════════════════════════════════════════════════
 * Common helpers
 * ═══════════════════════════════════════════════════════════════════════════ */

/** Turn the heading 90° clockwise. */
static inline Heading_t heading_right(Heading_t h)
{
    return (Heading_t)((h + 1U) & 3U);
}

/** Turn the heading 90° counter-clockwise. */
static inline Heading_t heading_left(Heading_t h)
{
    return (Heading_t)((h + 3U) & 3U);
}

/** Turn the heading 180°. */
static inline Heading_t heading_reverse(Heading_t h)
{
    return (Heading_t)((h + 2U) & 3U);
}

/** Return true when the timed action started at @p start_ms has finished.  */
static inline uint8_t action_complete(uint32_t start_ms, uint32_t duration_ms)
{
    return (HAL_GetTick() - start_ms) >= duration_ms ? 1U : 0U;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════════════════ */

void MazeSolver_Init(void)
{
    s_state   = MAZE_STATE_IDLE;
    s_heading = HEADING_NORTH;

    PID_Init(&s_wall_pid, PID_KP, PID_KI, PID_KD, -30.0f, 30.0f);
    PID_SetSetpoint(&s_wall_pid, WALL_SETPOINT_CM);

#ifdef MAZE_ALGO_FLOOD_FILL
    memset(s_walls, 0, sizeof(s_walls));
    s_pos_row = MAZE_ROWS - 1;   /* start at bottom-left cell               */
    s_pos_col = 0;
    flood_init();
    flood_update();
#endif

    Motor_Stop();
    printf("[MAZE] Initialised. Algorithm: %s\r\n",
#ifdef MAZE_ALGO_FLOOD_FILL
           "Flood-fill"
#else
           "Right-hand rule"
#endif
    );
}

void MazeSolver_Reset(void)
{
    Motor_Stop();
    PID_Reset(&s_wall_pid);
    MazeSolver_Init();
}

MazeState_t MazeSolver_GetState(void)   { return s_state;   }
Heading_t   MazeSolver_GetHeading(void) { return s_heading; }

/* ─────────────────────────────────────────────────────────────────────────
 * MazeSolver_Step  –  called every ~50 ms from main()
 * ───────────────────────────────────────────────────────────────────────── */
void MazeSolver_Step(void)
{
    /* Sample all distance sensors once per step                             */
    Ultrasonic_ReadAll();

    uint8_t wall_f = Ultrasonic_WallFront();
    uint8_t wall_l = Ultrasonic_WallLeft();
    uint8_t wall_r = Ultrasonic_WallRight();

    switch (s_state)
    {
    /* ── IDLE: wait for start (sensor check shows we are in the maze) ───── */
    case MAZE_STATE_IDLE:
        Motor_Stop();
        /* Transition: any wall detected means we are placed in the maze     */
        if (wall_f || wall_l || wall_r)
        {
            printf("[MAZE] Start detected – entering maze.\r\n");
            s_state          = MAZE_STATE_FORWARD;
            s_action_start_ms = HAL_GetTick();
        }
        break;

    /* ── FORWARD: drive straight with optional wall-following correction ── */
    case MAZE_STATE_FORWARD:
    {
#ifdef MAZE_ALGO_FLOOD_FILL
        /* ── Flood-fill: navigate toward decreasing flood values ─────────── */
        flood_learn_walls();
        flood_update();

        int8_t best = flood_best_direction();

        if (best < 0)
        {
            /* No accessible neighbour with a lower value – maze unsolvable  */
            printf("[MAZE] Flood-fill stuck – no path to goal.\r\n");
            Motor_Stop();
            s_state = MAZE_STATE_ERROR;
            break;
        }

        if (s_flood[s_pos_row][s_pos_col] == 0)
        {
            Motor_Stop();
            printf("[MAZE] Goal reached!\r\n");
            s_state = MAZE_STATE_SOLVED;
            break;
        }

        /* Apply PID side-correction while moving forward                    */
        float side_distance = wall_r
            ? (float)Ultrasonic_GetDistance(US_RIGHT)
            : (float)Ultrasonic_GetDistance(US_LEFT);
        float correction = PID_Compute(&s_wall_pid, side_distance);
        int16_t lspd = (int16_t)( MOTOR_BASE_SPEED + correction);
        int16_t rspd = (int16_t)( MOTOR_BASE_SPEED - correction);
        Motor_SetSpeed(lspd, rspd);

        /* After one cell traversal, decide next move                        */
        if (action_complete(s_action_start_ms, MAZE_CELL_DURATION_MS))
        {
            /* Advance position in the direction we travelled                */
            const int8_t dr[4] = { -1,  0,  1,  0 };
            const int8_t dc[4] = {  0,  1,  0, -1 };
            s_pos_row = (int8_t)(s_pos_row + dr[s_heading]);
            s_pos_col = (int8_t)(s_pos_col + dc[s_heading]);

            /* Determine next required heading                               */
            Heading_t target = (Heading_t)best;
            if (target == heading_right(s_heading))
            {
                s_state = MAZE_STATE_TURN_RIGHT;
            }
            else if (target == heading_left(s_heading))
            {
                s_state = MAZE_STATE_TURN_LEFT;
            }
            else if (target == heading_reverse(s_heading))
            {
                s_state = MAZE_STATE_UTURN;
            }
            else
            {
                /* Same heading – continue forward                            */
                s_action_start_ms = HAL_GetTick();
            }
            Motor_Brake();
            HAL_Delay(100U);
        }

#else /* ── Right-hand rule ──────────────────────────────────────────────── */

        /* Apply PID side-wall correction (prefer right wall if available)   */
        float side_distance;
        if (!wall_r)
            side_distance = (float)Ultrasonic_GetDistance(US_RIGHT);
        else if (!wall_l)
            side_distance = (float)Ultrasonic_GetDistance(US_LEFT);
        else
            side_distance = WALL_SETPOINT_CM;   /* no side reference       */

        float correction = PID_Compute(&s_wall_pid, side_distance);
        /* Positive correction → drift right → boost left, reduce right     */
        int16_t lspd = (int16_t)(MOTOR_BASE_SPEED + (int16_t)correction);
        int16_t rspd = (int16_t)(MOTOR_BASE_SPEED - (int16_t)correction);
        Motor_SetSpeed(lspd, rspd);

        /* Check for junction / wall after one cell duration                 */
        if (action_complete(s_action_start_ms, MAZE_CELL_DURATION_MS))
        {
            Motor_Brake();
            HAL_Delay(100U);

            /* Re-read sensors after stopping                                */
            Ultrasonic_ReadAll();
            wall_f = Ultrasonic_WallFront();
            wall_l = Ultrasonic_WallLeft();
            wall_r = Ultrasonic_WallRight();

            PID_Reset(&s_wall_pid);

            /* Right-hand rule priority                                      */
            if (!wall_r)
            {
                printf("[MAZE] Turn RIGHT\r\n");
                s_state           = MAZE_STATE_TURN_RIGHT;
                s_action_start_ms = HAL_GetTick();
            }
            else if (!wall_f)
            {
                printf("[MAZE] Straight ahead\r\n");
                s_state           = MAZE_STATE_FORWARD;
                s_action_start_ms = HAL_GetTick();
            }
            else if (!wall_l)
            {
                printf("[MAZE] Turn LEFT\r\n");
                s_state           = MAZE_STATE_TURN_LEFT;
                s_action_start_ms = HAL_GetTick();
            }
            else
            {
                printf("[MAZE] Dead end – U-turn\r\n");
                s_state           = MAZE_STATE_UTURN;
                s_action_start_ms = HAL_GetTick();
            }
        }
#endif /* MAZE_ALGO_FLOOD_FILL */
        break;
    }

    /* ── TURN_RIGHT: 90° clockwise in-place turn ─────────────────────────── */
    case MAZE_STATE_TURN_RIGHT:
        Motor_TurnRight(MOTOR_TURN_SPEED);
        if (action_complete(s_action_start_ms, MAZE_TURN_DURATION_MS))
        {
            Motor_Brake();
            HAL_Delay(100U);
            s_heading         = heading_right(s_heading);
            s_state           = MAZE_STATE_FORWARD;
            s_action_start_ms = HAL_GetTick();
            PID_Reset(&s_wall_pid);
        }
        break;

    /* ── TURN_LEFT: 90° counter-clockwise in-place turn ─────────────────── */
    case MAZE_STATE_TURN_LEFT:
        Motor_TurnLeft(MOTOR_TURN_SPEED);
        if (action_complete(s_action_start_ms, MAZE_TURN_DURATION_MS))
        {
            Motor_Brake();
            HAL_Delay(100U);
            s_heading         = heading_left(s_heading);
            s_state           = MAZE_STATE_FORWARD;
            s_action_start_ms = HAL_GetTick();
            PID_Reset(&s_wall_pid);
        }
        break;

    /* ── UTURN: 180° rotation ────────────────────────────────────────────── */
    case MAZE_STATE_UTURN:
        Motor_TurnRight(MOTOR_TURN_SPEED);
        if (action_complete(s_action_start_ms, MAZE_UTURN_DURATION_MS))
        {
            Motor_Brake();
            HAL_Delay(100U);
            s_heading         = heading_reverse(s_heading);
            s_state           = MAZE_STATE_FORWARD;
            s_action_start_ms = HAL_GetTick();
            PID_Reset(&s_wall_pid);
        }
        break;

    /* ── SOLVED: blink LED and stop ─────────────────────────────────────── */
    case MAZE_STATE_SOLVED:
        Motor_Stop();
        HAL_GPIO_TogglePin(LED_STATUS_PORT, LED_STATUS_PIN);
        HAL_Delay(200U);
        break;

    /* ── ERROR: stop and indicate fault ─────────────────────────────────── */
    case MAZE_STATE_ERROR:
        Motor_Stop();
        HAL_GPIO_WritePin(LED_STATUS_PORT, LED_STATUS_PIN, GPIO_PIN_SET);
        break;

    default:
        s_state = MAZE_STATE_IDLE;
        break;
    }
}
