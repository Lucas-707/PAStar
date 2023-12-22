# Parallel A* Search Project

### Final Report: https://katrina0406.github.io/parallel_astar_search/assets/15418-Final-Report.pdf

### Project Website: https://katrina0406.github.io/parallel_astar_search/

### 1. Sequential A* Search Benchmark

### 2. HDA* Search Algorithm
reference paper: https://ojs.aaai.org/index.php/ICAPS/article/view/13350/13198

### 3. GA* Search Algorithm
reference paper: https://cdn.aaai.org/ojs/9367/9367-13-12895-1-2-20201228.pdf

### 4. Command to run:

./build_debug/pastar --seed=0 --map=benchmark/empty-16-16.map --agents=benchmark/empty-16-16-random-1.scen --output=test.csv  --outputPaths=test_path.txt --algo="A*" --trialNum=1


./build_debug/pastar --seed=0 --map=benchmark/Boston_0_1024.map --agents=benchmark/Boston_0_1024.map.scen --output=test.csv  --outputPaths=test_path.txt --algo="A*" --trialNum=1


mpirun -np 4 ./build_debug/pastar --seed=0 --map=benchmark/Boston_0_1024.map --agents=benchmark/Boston_0_1024.map.scen --output=test.csv  --outputPaths=test_path.txt --algo="HDA*" --trialNum=1 --debugwait=0
