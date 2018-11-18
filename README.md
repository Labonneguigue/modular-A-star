# Modular Implementation of the A* algorithm

The goal here is to create a modular A* algorithm library which provides an interface for how the state should be defined and then resolves the path planning problem.

### Results

#### Non-holonomic robot motion.

From start (`?`) to finish (`@`).

Each visited locations are marked `*`. The inaccessible locations are `X`.
Some locations seam "jumped" because the speed of the robot is greater than the size of each cells per iterations.


```bash
Found path to goal in 12904 expansions.

 ? X X 0 0 0 0 0 0 0 X X 0 0 0 0 0 X X 0 0 0 0 0 0 0 0 0 0 0 0 0 0
 * X X 0 0 0 0 0 0 X X 0 0 0 0 0 0 X X 0 0 0 0 0 0 0 0 0 0 0 0 0 0
 * X X 0 0 0 0 0 X X 0 0 * * 0 0 0 X X 0 0 0 0 0 0 0 0 0 0 0 0 0 0
 0 X X 0 0 0 0 X X 0 0 * X X X 0 0 X X 0 0 0 0 0 0 0 0 0 0 0 0 0 0
 * X X 0 0 0 X X 0 0 * X X X * 0 0 X X 0 0 0 0 0 0 0 0 0 0 0 0 0 0
 * X X 0 0 X X 0 0 * X X X * 0 0 0 X X 0 0 0 0 0 0 0 0 0 0 0 0 0 0
 * X X 0 X X 0 0 * X X X * * 0 0 0 X X 0 0 0 0 0 0 0 0 0 0 0 0 0 0
 0 X X X X 0 0 * X X X 0 * 0 0 0 0 X X 0 0 0 0 0 0 0 0 0 0 0 0 0 0
 * X X X 0 0 * X X X 0 * 0 0 0 0 0 X X 0 0 0 0 0 0 0 0 0 0 0 0 0 0
 * X X 0 0 * X X X 0 * X X X X X X X X 0 0 0 0 0 0 0 0 0 0 0 0 0 0
 0 X * 0 * X X X 0 * X X X X X X X X X 0 0 0 0 0 0 0 0 0 0 0 0 0 0
 0 0 0 * X X X 0 * X X X X X X X X X X 0 0 0 0 0 0 0 0 0 0 0 0 0 0
 0 0 0 X X X 0 * X X X X X X X X X X X 0 0 0 0 0 0 0 0 0 0 0 0 0 0
 0 0 X X X 0 0 X X X X X X X X X X X X 0 0 0 0 0 * * X * * 0 0 0 0
 0 X X X 0 0 0 0 * 0 0 0 0 0 0 0 0 0 0 0 0 0 0 * 0 X X X 0 * 0 0 0
 X X X 0 0 0 0 0 0 * * * 0 * * 0 * * * 0 * * 0 * X X X X X 0 * * @
```