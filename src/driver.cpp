﻿/* Copyright (C) Jiaoyang Li
* Unauthorized copying of this file, via any medium is strictly prohibited
* Confidential
* Written by Jiaoyang Li <jiaoyanl@usc.edu>, May 2020
*/

/*driver.cpp
* Solve a MAPF instance on 2D grids.
*/
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <unistd.h>
#include "SpaceTimeAStar.h"
#include "HDAStar.h"

/* Main function */
int main(int argc, char** argv)
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		
		("seed", po::value<int>()->default_value(0), "random seed")
		// params for the input instance and experiment settings
		("map,m", po::value<string>()->required(), "input file for map")
		("agents,a", po::value<string>()->required(), "input file for start/goals")
		("output,o", po::value<string>(), "output file for statistics")
		("outputPaths", po::value<string>(), "output file for paths")
		("algo", po::value<string>()->default_value("A*"), "algorithm of planner (A*, HDA*)")
		("threads,t", po::value<int>()->default_value(1), "number of threads to use")
		("trialNum,k", po::value<int>()->default_value(1), "number of trials")
		("cutoffTime", po::value<double>()->default_value(60), "cutoff time (seconds)")
		("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
		("debugwait", po::value<int>()->default_value(0), "wait for 5 secs for vscode debugger")
		("enlarge", po::value<int>()->default_value(1), "enlarge the map by this factor")
		;
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		cout << desc << endl;
		return 1;
	}

	po::notify(vm);

	int theSeed = vm["seed"].as<int>();
	srand(theSeed);

	///////////////////////////////////////////////////////////////////////////
	// load the instance
	Instance instance(vm["map"].as<string>(), vm["agents"].as<string>(),
		vm["trialNum"].as<int>(), vm["enlarge"].as<int>());
	//////////////////////////////////////////////////////////////////////
    // initialize the solver
	if (vm["algo"].as<string>() == "A*")
	{
		for (int i=0; i < vm["trialNum"].as<int>(); i++) {
			Timer timer;
			SpaceTimeAStar* planner = new SpaceTimeAStar(instance, i);
			// planner->heuristics_time += timer.elapsed();
			// Timer path_timer;
			Path path = planner->findOptimalPath();
			// planner->path_finding_time = path_timer.elapsed();
			float runtime = timer.elapsed();
			planner->runtime = runtime; 
			if (vm.count("output"))
				planner->saveResults(vm["output"].as<string>(), vm["agents"].as<string>());
			if (vm.count("outputPaths"))
				planner->savePaths(vm["outputPaths"].as<string>());
			delete planner;
		}
	}
	else if (vm["algo"].as<string>() == "HDA*")
	{	
		if (vm["debugwait"].as<int>())
			sleep(15);


		// Initialize MPI
		int pid, nproc;
		MPI_Init(&argc, &argv);
		MPI_Comm_rank(MPI_COMM_WORLD, &pid);
		MPI_Comm_size(MPI_COMM_WORLD, &nproc);
		for (int i=0; i < vm["trialNum"].as<int>(); i++) {
			MPI_Barrier(MPI_COMM_WORLD);

			Timer timer;
			HDAStar* planner = new HDAStar(instance, i, nproc, pid);
			// planner->heuristics_time += timer.elapsed();
			// Timer path_timer;
			Path path = planner->findOptimalPath();
			// planner->path_finding_time = path_timer.elapsed();
			MPI_Barrier(MPI_COMM_WORLD);

			if (pid == 0) { // should be the process that find goal
				float runtime = timer.elapsed();
				planner->runtime = runtime; 
			}

			// gather the total node from all process
			int recv_node_gen[nproc]; int recv_node_exp[nproc];
			MPI_Gather(&planner->num_generated, 1, MPI_INT, &recv_node_gen, 1, MPI_INT, 0, MPI_COMM_WORLD);
			MPI_Gather(&planner->num_expanded, 1, MPI_INT, &recv_node_exp, 1, MPI_INT, 0, MPI_COMM_WORLD);
			planner->num_expanded = 0; planner->num_generated = 0;
			for (int p=0; p<nproc; p++) {
				planner->num_expanded += recv_node_exp[p];
				planner->num_generated += recv_node_gen[p];
			}
			
			if (pid == 0) {
				if (vm.count("output"))
					planner->saveResults(vm["output"].as<string>(), vm["agents"].as<string>());
				// if (vm.count("outputPaths"))
				// 	planner->savePaths(vm["outputPaths"].as<string>());
			}
			delete planner;
			
		}
		MPI_Finalize();
	}

	


	return 0;

}