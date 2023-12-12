# Parallel A* Search Project

### 1. Sequential A* Search Benchmark

### 2. HDA* Search Algorithm
reference paper: https://ojs.aaai.org/index.php/ICAPS/article/view/13350/13198



#### Command to run:

./build_debug/pastar --seed=0 --map=benchmark/empty-16-16.map --agents=benchmark/empty-16-16-random-1.scen --output=test.csv  --outputPaths=test_path.txt --algo="A*" --trialNum=1


./build_debug/pastar --seed=0 --map=benchmark/maze512-4-6.map --agents=benchmark/maze512-4-6.map.scen --output=test.csv  --outputPaths=test_path.txt --algo="A*" --trialNum=1


mpirun -np 4 ./build_debug/pastar --seed=0 --map=benchmark/maze512-4-6.map --agents=benchmark/maze512-4-6.map.scen --output=test.csv  --outputPaths=test_path.txt --algo="HDA*" --trialNum=1 --debugwait=0
