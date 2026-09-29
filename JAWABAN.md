# KPP Pelatdas Programming 2026 - Answers

**Robot name:** Atlas  
**Algorithm:** A* (Manhattan heuristic), run twice per mission: S → F, then F → G  
**Coordinates:** `(row,col)`, zero-indexed, top-left is `(0,0)` (same as the sheet's example)

Build and run:

```
g++ -std=c++17 maze_solver.cpp -o maze_solver
maze_solver              # all 3 mazes, map printed after every step
maze_solver --summary    # summary only (format from the sheet)
maze_solver 2            # only Soal 2
maze_solver 3 --color    # ANSI colours   |  --animate = 300 ms delay per step
```

---

## Soal 1 (7 x 7)

```
# # # # # # #
# S # . . . #
# . # . # G #
# . . . # . #
# # # . . . #
# X . . F . #
# # # # # # #
```

| | |
|---|---|
| Start | (1,1) |
| Flag | (5,4) |
| Base | (2,5) |
| **Path to flag** | DOWN, DOWN, RIGHT, RIGHT, DOWN, RIGHT, DOWN |
| **Flag captured** | (5,4) |
| **Path to base** | UP, RIGHT, UP, UP |
| **Total moves** | **11** |

## Soal 2 (9 x 9)

```
# # # # # # # # #
# S . . # . . . #
# . # . # . # G #
# . # . . . # . #
# . . . # . . . #
# # # . # . . # #
# X . . # . . F #
# . # # # X . . #
# # # # # # # # #
```

| | |
|---|---|
| Start | (1,1) |
| Flag | (6,7) |
| Base | (2,7) |
| **Path to flag** | RIGHT, RIGHT, DOWN, DOWN, RIGHT, RIGHT, DOWN, RIGHT, DOWN, DOWN, RIGHT |
| **Flag captured** | (6,7) |
| **Path to base** | LEFT, UP, UP, RIGHT, UP, UP |
| **Total moves** | **17** |

## Soal 3 (11 x 11)

```
# # # # # # # # # # #
# S . . # . . . . . #
# . # . # . # # # . #
# . # . . . # G # . #
# . . . # . # . # . #
# # # . # . . . # . #
# . . . # # # . # . #
# . # . . X . . # . #
# . # . # # # . # F #
# X . . . . X . . . #
# # # # # # # # # # #
```

| | |
|---|---|
| Start | (1,1) |
| Flag | (8,9) |
| Base | (3,7) |
| **Path to flag** | RIGHT, RIGHT, DOWN, DOWN, RIGHT, RIGHT, DOWN, DOWN, RIGHT, RIGHT, DOWN, DOWN, DOWN, DOWN, RIGHT, RIGHT, UP |
| **Flag captured** | (8,9) |
| **Path to base** | DOWN, LEFT, LEFT, UP, UP, UP, UP, UP, UP |
| **Total moves** | **26** |

---

## How the program works

1. **Map storage.** Each maze is copied into a real 2D `char` array named `mazeGridBuffer`, sized exactly 7x7, 9x9 or 11x11 (`Maze<N>` is a subclass of `MazeBase`).
2. **Finding S, F, G.** `methodLocate()` scans the array for the symbols. No coordinates are hard-coded.
3. **Legal tiles.** `methodWalkable()` allows `S F G .` and rejects `#`, `X` and anything outside the map.
4. **A\* search.** `AStarPathfinder` keeps its frontier in `q_traversalNode` (a min-heap ordered by `f = g + Manhattan distance`) and rebuilds the route from parent links.
5. **Order S → F → G.** The search runs twice: S to F, then F to G. G is an ordinary walkable tile while the flag is not yet taken, so the robot may cross it. The mission only counts as complete if the flag was picked up and the robot ends on G.
6. **Execution.** `AutoRobot` (subclass of `Robot`) calls `methodMove()` for each direction. `Viz()` reprints the map after every step with the robot shown as `R`. Once the flag is taken its `F` tile is drawn as `.`.
7. **Verification.** Both routes were checked with an independent BFS over (position, has-flag) states. All three answers are optimal, and the sheet's own 7x7 example reproduces its expected output (12 moves).

## Class layout

```
MazeBase  <-  Maze<N>          (owns mazeGridBuffer)
Pathfinder <- AStarPathfinder  (owns q_traversalNode during search)
Robot     <-  AutoRobot        (methodExecuteMission)
Compass                        (static direction / distance / formatting helpers)
Viz()                          (free function: map visualisation)
```
