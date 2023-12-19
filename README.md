# Solving AMAPF problem


Solve AMAPF problem using maximum flow with Bulk-Search algorithm for finding the paths of the flow.

The code is written with 'light' level of comments. However, the algorithm works as described in the paper.

## Folders
1. `data` contains the maps, scenarios and file `estimate.txt` which contins the lower-bounds of the makespan of each map/scenario (found by algorithm BOTTLENECK from this [library](https://github.com/Kei18/tswap/blob/master/unlabeled_mapf/src/flow_network.cpp)).
2. `results` where to store the results.

## Know before run
In this package, three ways for iterating until finding the optimal (minimal) height of the network, are supported:
1. Iterating from 1 (the default method).
2. Iterating from the value in file `data/estimate.txt` (can be enabled by flag `-s`).
3. Binary search (can be enabled by flag `-b`).

## Build and Run
The package was tested and run on `Ubuntu 22.04.2`.\
Build the package by:
```
mkdir build
cd build
cmake ..
make
```

All tests used in the paper (all mapf maps, all scenarios, the number of agents begin from 1 and increase by *2), are hard-coded in the main function, and ready to run.

Run the program by one of these three commands:\
either
```
./flow_bs 
```
or (for iterating from the value stored in file `data/estimate.txt`)
```
./flow_bs -s
```
or (for binary search)
```
./flow_bs -b # for 
```

## Results
The makespan of each instance are logged in a csv-style file (`results/results*2.csv`). The results are also printed in the terminal.
## TODO
- Implement function to print the paths (which form the mimimum makespan) for each agent.
