# ResQRoute — Full-stack C++ Emergency Route Planner

A college-demo full-stack DSA project: C++17 HTTP API backend + HTML/CSS/JavaScript responsive frontend.

## Features
- Dijkstra and A* shortest-time routing
- Adjacency matrix and adjacency list representations
- AVL tree hospital directory indexed by specialty
- Nearest matching hospital by simulated route time
- Benchmark comparison: elapsed microseconds, expanded nodes, route cost
- Simulated road-time updates API
- Schematic interactive city map, route highlighting, responsive dashboard
- No third-party C++ libraries required

> Important: all locations and travel times are fictional. This is an academic simulation, not a live navigation service and must not be used for real emergency dispatch.

## Windows setup (VS Code)
1. Extract the ZIP.
2. Open the `EmergencyRoutePlanner_FullStack` folder in VS Code.
3. In the terminal, check `g++ --version`.
4. Compile:
   `g++ -std=c++17 -O2 -Wall -Wextra server.cpp -o server.exe -lws2_32`
5. Run:
   `.\server.exe`
6. Open `http://localhost:8080` in your browser.

If you use Visual Studio/MSVC instead of g++, Winsock is available but compile flags differ. Recommended command above is for MinGW-w64 g++.

## API endpoints
- `GET /api/locations`
- `GET /api/route?start=0&goal=6&algorithm=dijkstra&representation=list`
- `GET /api/route?start=0&goal=6&algorithm=astar&representation=matrix`
- `GET /api/hospitals?specialty=cardiology`
- `GET /api/benchmark?start=0&goal=6`
- `POST /api/road?from=1&to=2&minutes=8` (updates simulated network; `minutes=0` closes it)

The frontend and backend are served by the same C++ process, so no separate frontend server is required.

## Folder structure
```
EmergencyRoutePlanner_FullStack/
  server.cpp
  public/
    index.html
    style.css
    app.js
  README.md
```

## DSA notes for presentation
- Dijkstra finds shortest paths with nonnegative edge costs.
- A* uses a heuristic to guide search. For a formal optimality guarantee, the heuristic must not overestimate the remaining travel time.
- AVL is a self-balancing binary search tree. This demo stores hospitals using a composite specialty/name/location key and traverses the tree to collect matching specialties.
- Matrix takes O(V^2) memory; adjacency list takes O(V+E) memory.
- Benchmark times on a tiny graph are noisy; use larger generated graphs and repeat queries for a serious performance report.
