# Parallel A* Search Project

### 1. Sequential A* Search Benchmark

### 2. HDA* Search Algorithm
reference paper: https://ojs.aaai.org/index.php/ICAPS/article/view/13350/13198



#### Command to run:

./build_debug/pastar --seed=0 --map=benchmark/empty-16-16.map --agents=benchmark/empty-16-16-random-1.scen --output=test.csv  --outputPaths=test_path.txt --algo="A*" --trialNum=1


./build_debug/pastar --seed=0 --map=benchmark/Boston_0_1024.map --agents=benchmark/Boston_0_1024.map.scen --output=test.csv  --outputPaths=test_path.txt --algo="A*" --trialNum=1 --enlarge=2

./build_debug/pastar --seed=0 --map=benchmark/Boston_0_256.map --agents=benchmark/Boston_0_256.map.scen --output=test.csv  --outputPaths=test_path.txt --algo="A*" --trialNum=7 --hash="random"

mpirun -np 4 ./build_release/pastar --map=benchmark/Paris_1_1024.map --agents=benchmark/Paris_1_1024.map.scen --output=test.csv  --outputPaths=test_path.txt --algo="HDA*" --trialNum=7 --enlarge=1 --hash="grid" --debugwait=0

mpirun -np 4 ./build_release/pastar --seed=0 --map=benchmark/den312d.map --agents=benchmark/den312d.map.scen --output=test.csv  --outputPaths=test_path.txt --algo="HDA*" --trialNum=10 --debugwait=0

