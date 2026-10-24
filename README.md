# Optimized Route Planner for Emergency Vehicles (C++)

## Included DSA features
- Dijkstra's shortest path algorithm
- A* Search
- Adjacency Matrix and Adjacency List graph representations
- AVL Tree indexing hospital records by specialty
- Performance comparison (elapsed microseconds, explored nodes, route cost)
- Simulated traffic-time changes and road closures
- Sample city network with hospital and patient locations

> This is an educational simulation with fictional locations and travel times. It is not connected to live traffic or emergency dispatch and must not be used to direct real ambulances.

## Requirements
- A C++17 compiler (g++/MinGW-w64 recommended) and VS Code, or Visual Studio.
- No third-party libraries required.

## Run on Windows (VS Code terminal)
1. Extract the ZIP.
2. Open the `EmergencyRoutePlannerCpp` folder in VS Code.
3. Open Terminal > New Terminal.
4. Check the compiler: `g++ --version`
5. Compile:
   `g++ -std=c++17 -O2 -Wall -Wextra main.cpp -o route_planner.exe`
6. Run:
   `.\route_planner.exe`

If `g++` is not recognized, install/configure a C++ compiler such as MinGW-w64, then reopen the terminal.

## Menu
1. Dijkstra using adjacency list
2. A* using adjacency list
3. Search hospitals by specialty using AVL Tree and route to the nearest matching one
4. Compare Dijkstra/A* with matrix/list representations
5. Change a road's travel time or close it (enter 0)
6. List location IDs

## Important implementation notes
- The city coordinates are illustrative, not GPS.
- The A* heuristic is deliberately conservative for this sample network. For a real road network, use a lower-bound estimate of travel time, such as straight-line distance divided by a verified maximum speed, so it does not overestimate.
- Benchmarks are small and noisy. For a meaningful report, generate larger sparse and dense graphs, run each query many times, and report averages.
- The AVL tree stores each hospital under a composite key containing specialty, name and location. The current demonstration traverses the tree to collect all hospitals of a specialty; an optimized specialty index could store a list at each specialty key.
- Road closures are represented by infinite travel time.
