/**
 * @file  maze_solver.h
 * @brief Maze-solving algorithms for the STM32F103VET6 Maze Car.
 *
 * Two algorithms are available, selected by the MAZE_ALGO compile-time macro:
 *
 *   MAZE_ALGO_RIGHT_HAND (default)
 *   ───────────────────────────────
 *   Classic wall-following (right-hand rule).  The car always keeps its
 *   right hand touching the right wall.  Works for any simply-connected
 *   (loop-free) maze and is O(1) in memory.
 *
 *   Priority at each cell:
 *     1. Turn right and move forward if the right corridor is open.
 *     2. Go straight if the front corridor is open.
 *     3. Turn left if the left corridor is open.
 *     4. Turn 180° (U-turn) if all three are blocked (dead end).
 *
 *   MAZE_ALGO_FLOOD_FILL
 *   ─────────────────────
 *   Flood-fill (distance-label) algorithm.  Builds a Manhattan-distance map
 *   of the maze from the goal cell and navigates toward decreasing distance
 *   values.  Always finds the shortest path but requires a MAZE_COLS×MAZE_ROWS
 *   array in SRAM.
 *
 * Usage
 * ─────
 *   MazeSolver_Init();         // call once after peripherals are ready
 *   for (;;) {
 *       MazeSolver_Step();     // call every control loop iteration (~50 ms)
 *   }
 */

#ifndef __MAZE_SOLVER_H
#define __MAZE_SOLVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ── Algorithm selection ─────────────────────────────────────────────────── */
#ifndef MAZE_ALGO_FLOOD_FILL
  #define MAZE_ALGO_RIGHT_HAND   1   /* enable right-hand rule (default)    */
#endif

/* ── Maze dimensions (flood-fill only) ──────────────────────────────────── */
#define MAZE_COLS   16
#define MAZE_ROWS   16

/* ── Timing constants ────────────────────────────────────────────────────── */
#define MAZE_TURN_DURATION_MS    500U  /* time allowed for a 90° turn (ms)  */
#define MAZE_CELL_DURATION_MS    600U  /* time to cross one cell (ms)       */
#define MAZE_UTURN_DURATION_MS  1000U  /* time for a 180° U-turn (ms)       */
#define MAZE_SENSOR_INTERVAL_MS   50U  /* distance sampling period (ms)     */

/* ── Heading ─────────────────────────────────────────────────────────────── */
typedef enum {
    HEADING_NORTH = 0,
    HEADING_EAST  = 1,
    HEADING_SOUTH = 2,
    HEADING_WEST  = 3
} Heading_t;

/* ── Solver state (right-hand rule) ─────────────────────────────────────── */
typedef enum {
    MAZE_STATE_IDLE       = 0,
    MAZE_STATE_FORWARD    = 1,
    MAZE_STATE_TURN_RIGHT = 2,
    MAZE_STATE_TURN_LEFT  = 3,
    MAZE_STATE_UTURN      = 4,
    MAZE_STATE_SOLVED     = 5,
    MAZE_STATE_ERROR      = 6
} MazeState_t;

/**
 * @brief Initialise the maze solver.  Call after Motor_Init() and
 *        Ultrasonic_Init().
 */
void MazeSolver_Init(void);

/**
 * @brief Execute one iteration of the maze-solving loop.
 *        Should be called repeatedly from main().
 */
void MazeSolver_Step(void);

/**
 * @brief Return the current solver state.
 */
MazeState_t MazeSolver_GetState(void);

/**
 * @brief Return the current logical heading.
 */
Heading_t MazeSolver_GetHeading(void);

/**
 * @brief Reset the solver to its initial state (e.g. to re-run the maze).
 */
void MazeSolver_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAZE_SOLVER_H */
