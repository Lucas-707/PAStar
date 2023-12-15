import os
import sys
import time
from datetime import datetime # For printing current datetime
import subprocess # For executing c++ executable
import numpy as np
import argparse

csv_path = "./results/hda_rst.csv"
benchmark_dir = "./benchmark"
nproc_list = [1, 2, 4, 8, 16, 32, 64, 128]
trial_num = 10

map_list = []
for file in os.listdir(benchmark_dir):
    if file[-3:] == 'map':
        # print(file)
        map_list.append(os.path.join(benchmark_dir, file))
map_list = sorted(map_list)

map_list = [
    # "maze512-1-9.map",
    # "maze512-4-6.map",
    # "maze512-8-4.map",
    # "maze512-32-9.map",
    # "Paris_1_256.map",
    # "Paris_1_512.map",
    # "Paris_1_1024.map",
    # "random512-40-5.map",
    # "den201d.map",
    # "den312d.map",
    # "den602d.map",
    # "orz900d.map",
    # "64room_005.map",
]


for map in map_list:
    csv_path = f"./results/{map[:-4]}.csv"
    print(f"start running experiments on {map}")
    map = "./benchmark/" + map
    scen = map + ".scen"
    
    command = "./build_release/pastar"
    command += " --seed=0"
    command += f" --map={map}"
    command += f" --agents={scen}"
    command += f" --output={csv_path}"
    command += f" --trialNum={trial_num}"
    seq_command = command + " --algo=A*"
    hda_command = command + " --algo=HDA*"

    #run sequential
    subprocess.run(seq_command.split(" "), check=True) # True if want failure error

    #run HDA*
    for np in nproc_list:
        mpi_command = f"mpirun -np {np} " + hda_command
        print("mpi command: \n",mpi_command)
        subprocess.run(mpi_command.split(" "), check=True)

    print(f"finished running experiments on {map}")

'''
mpirun -np 128 ./build_release/pastar --seed=0 --map=./benchmark/Paris_1_256.map --agents=./benchmark/Paris_1_256.map.scen --output=./results/Paris_1_256.csv --trialNum=10 --algo=HDA*
mpirun -np 16 ./build_release/pastar --seed=0 --map=./benchmark/den201d.map --agents=./benchmark/den201d.map.scen --output=./results/den201d.csv --trialNum=10 --algo=HDA*

mpirun -np 32 ./build_release/pastar --seed=0 --map=./benchmark/orz900d.map --agents=./benchmark/orz900d.map.scen --output=./results/orz900d.csv --trialNum=10 --algo=HDA*
'''