import os
import sys
import time
from datetime import datetime # For printing current datetime
import subprocess # For executing c++ executable
import numpy as np
import argparse

csv_path = "./results/hda_rst.csv"
benchmark_dir = "./benchmark"
nproc_list = [1, 2, 4]
trial_num = 10



map_list = []
for file in os.listdir(benchmark_dir):
    if file[-3:] == 'map':
        # print(file)
        map_list.append(os.path.join(benchmark_dir, file))
map_list = sorted(map_list)

for map in map_list:
    print(f"start running experiments on {map}")
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

