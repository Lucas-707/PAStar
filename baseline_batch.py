import os
import sys
import time
from datetime import datetime # For printing current datetime
import subprocess # For executing c++ executable
import numpy as np
import argparse
import pdb
import pandas as pd
from os.path import exists

class BatchRunner:
    """Class for running a single scen file"""
    def __init__(self, bypass : bool, target : bool, subopt : float, cutoffTime, 
                output, path_prefix, mapfile, scenfile) -> None:
        self.bypass = bypass
        self.target = target
        self.cutoffTime = cutoffTime
        self.output = output
        self.subopt = subopt
        self.outputCSVFile = output + ".csv"
        self.path_prefix = path_prefix
        self.mapfile = mapfile
        self.scenfile = scenfile

    def runSingleSettingsOnMap(self, numAgents, aSeed):
        # Main command
        command = "./build_release/eecbs"

        ## Batch experiment settings
        command += " --seed={}".format(aSeed)
        command += " --agentNum={}".format(numAgents)
        command += " --map={}".format(self.mapfile)
        command += " --agents={}".format(self.scenfile)
        # command += " --outputCSVFile={}".format(self.outputCSVFile)
        command += " --output={}".format(self.outputCSVFile)
        command += " --outputPaths={}".format(self.path_prefix + "-base-k" + str(numAgents) + "-s" + str(aSeed) + ".txt")
        ## Exp Settings
        command += " --suboptimality={}".format(self.subopt)
        command += " --cutoffTime={}".format(self.cutoffTime)
        command += " --bypass={}".format(self.bypass)
        command += " --targetReasoning={}".format(self.target)

        subprocess.run(command.split(" "), check=True) # True if want failure error

    def detectExistingStatus(self, numAgents):
        if exists(self.outputCSVFile):
            df = pd.read_csv(self.outputCSVFile)
            
            # pdb.set_trace()
            df = df[(df["instance name"] == self.scenfile) &
                     (df["#agents"] == numAgents)
                    ]
            numFailed = len(df[(df["solution cost"] <= 0) | (df["solution cost"] >= 1073741823)])
            return len(df), numFailed
        return 0, 0

    def runBatchExps(self, agent_range, seeds):
            for aNum in agent_range:
                numRan, numFailed = self.detectExistingStatus(aNum)
                if numFailed > len(seeds) / 2: ## Check if existing run all failed
                    print("Terminating early because all failed with #agents = {}".format(aNum))
                    break
                elif numRan >= len(seeds): ## Check if ran existing run
                    print("Skipping {} completely as already run!".format(aNum))
                    continue
                else:
                    for aSeed in seeds:
                        numRan, numFailed = self.detectExistingStatus(aNum)
                        if numFailed > len(seeds) / 2:
                            break
                        self.runSingleSettingsOnMap(aNum,aSeed)
                    #### Check if new run failed
                    numRan, numFailed = self.detectExistingStatus(aNum)
                    if numFailed > len(seeds) / 2:
                        print("Terminating early because all failed with #agents = {}".format(aNum))
                        break

def runExps():
    batchFolderName = "rebuttal"
    mapname = sys.argv[1]
    scen_id = sys.argv[2]
    
    mapsToNumAgents = {
    "Paris_1_256": (100, 400, 100), # Verified
    "random-32-32-20": (10, 150, 10), # Verified this
    #"random-32-32-10": (50, 461), # Verified
    #"den520d": (50, 1000), # Verified this
    "den312d": (25, 250, 25), # Verified
    #"empty-32-32": (10, 80), # Verified
    "empty-48-48": (50, 600, 25), # Verified this
    "ht_chantry": (50, 400, 25), # Verified
    #"maze-32-32-4": (25, 500), # Verified
    #"room-32-32-4": (25, 500), # Verified
    #"room-64-64-8": (25, 500), # Verified
    #"lak303d": (25, 500), # Verified
}

    seeds = [1,2,3,4,5]

    ## Make folder if does not exist
    if not os.path.isdir(batchFolderName):
        os.makedirs(batchFolderName)
    
    agent_range = range(mapsToNumAgents[mapname][0], mapsToNumAgents[mapname][1]+1, mapsToNumAgents[mapname][2])
    expSettings = dict(
    bypass = 1,
    target = 1,
    subopt = 1.5,
    cutoffTime = 120,
    output = batchFolderName + "/baseline_stats",
    path_prefix = batchFolderName + "/paths/" + mapname + "-" + scen_id,
    mapfile = "benchmark/mapf-map/" + mapname +".map",
    scenfile = "benchmark/scen-random/" + mapname + "-random-1.scen"
    )
    myBR = BatchRunner(**expSettings)
    myBR.runBatchExps(agent_range, seeds)


if __name__ == "__main__":
    runExps()