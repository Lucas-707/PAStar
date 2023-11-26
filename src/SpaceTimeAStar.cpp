#include "SpaceTimeAStar.h"


void SpaceTimeAStar::updatePath(const LLNode* goal, vector<PathEntry> &path)
{
    const LLNode* curr = goal;
    if (curr->is_goal)
        curr = curr->parent;
    path.reserve(curr->g_val + 1);
    while (curr != nullptr)
    {
        path.emplace_back(curr->location);
        curr = curr->parent;
    }
    std::reverse(path.begin(),path.end());
}


Path SpaceTimeAStar::findOptimalPath()
{
    return findSuboptimalPath();
}

// find path by time-space A* search
// Returns a bounded-suboptimal path that satisfies the constraints of the give node  while
// minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
// lowerbound is an underestimation of the length of the path in order to speed up the search.
Path SpaceTimeAStar::findSuboptimalPath()
{
    Path path;
    num_expanded = 0;
    num_generated = 0;

    // generate start and add it to the OPEN & FOCAL list
    auto start = new AStarNode(start_location, 0, my_heuristic[start_location], nullptr, 0, 0);

    pushNode(start);
    allNodes_table.insert(start);
    min_f_val = (int) start->getFVal();
    // lower_bound = int(w * min_f_val));

    while (!open_list.empty())
    {
        auto* curr = popNode();
        assert(curr->location >= 0);
        // check if the popped node is a goal
        if (curr->location == goal_location) // arrive at the goal location
        {
            updatePath(curr, path);
            break;
        }

        auto next_locations = instance.getNeighbors(curr->location);
        next_locations.emplace_back(curr->location);
        for (int next_location : next_locations)
        {
            int next_timestep = curr->timestep + 1;
            // compute cost to next_id via curr node
            int next_g_val = curr->g_val + 1;
            int next_h_val = my_heuristic[next_location];
            
            // generate (maybe temporary) node
            auto next = new AStarNode(next_location, next_g_val, next_h_val,
                                      curr, next_timestep);
            
            // try to retrieve it from the hash table
            auto it = allNodes_table.find(next);
            if (it == allNodes_table.end())
            {
                // not in hash table
                pushNode(next);
                allNodes_table.insert(next);
                continue;
            }

            // update existing node's if needed (only in the open_list)
            auto existing_next = *it;
            if (existing_next->getFVal() > next->getFVal()) // if f-val decreased through this new path
            {
                if (!existing_next->in_openlist) // if it is in the closed list (reopen)
                {
                    existing_next->copy(*next);
                    pushNode(existing_next);
                }
                else
                {
                    bool update_open = false;
                    if (existing_next->getFVal() > next->getFVal())
                        update_open = true;

                    existing_next->copy(*next);	// update existing node

                    if (update_open)
                        open_list.increase(existing_next->open_handle);  // increase because f-val improved
                }
            }

            delete(next);  // not needed anymore -- we already generated it before
        }  // end for loop that generates successors
    }  // end while loop

    releaseNodes();
    planned_path = path;
    path_cost = path.size() - 1;
    return path;
}


inline AStarNode* SpaceTimeAStar::popNode()
{
    auto node = open_list.top(); open_list.pop();
    // open_list.erase(node->open_handle);
    node->in_openlist = false;
    num_expanded++;
    return node;
}


inline void SpaceTimeAStar::pushNode(AStarNode* node)
{
    node->open_handle = open_list.push(node);
    node->in_openlist = true;
    num_generated++;
}




void SpaceTimeAStar::releaseNodes()
{
    open_list.clear();
    for (auto node: allNodes_table)
        delete node;
    allNodes_table.clear();
}
