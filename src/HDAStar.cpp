#include "HDAStar.h"


void HDAStar::updatePath(const LLNode* goal, vector<PathEntry> &path)
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


Path HDAStar::findOptimalPath()
{
    return findSuboptimalPath();
}

int HDAStar::hash(const LLNode* node)
{
    // need to update the hash function
    return node->location % nproc;
}


void HDAStar::create_msg_mpi_datatype()
{
    MPI_Type_contiguous(sizeof(msg), MPI_BYTE, &MPI_Msg);
    MPI_Type_commit(&MPI_Msg);
}

HDAStar::msg HDAStar::create_msg(AStarNode* node)
{
    struct msg msg_;
    msg_.node = node;
    return msg_;
}

void HDAStar::clear_message_set()
{
    for(int i = 0; i < message_set.size(); i++)
        message_set[i].clear();
}


void HDAStar::send_message_set()
{
    int is_complete;
    for(int i = 0; i < message_set.size(); i++)
    {
        if(i == pid)
            continue;
        if(send_requests[i] != nullptr) //check if previous send is pending
        {
            MPI_Test(send_requests[i], &is_complete, MPI_STATUS_IGNORE);
            if(!is_complete)continue; //send is still executing
            send_buffers[i].clear(); //empty send buffer
            send_requests[i] = nullptr;
        }
        if( !(message_set[i].empty()) )
        {
            send_requests[i] = new MPI_Request;
            send_buffers[i].assign(message_set[i].begin(), message_set[i].end()); //copy data into send buffer   
            MPI_Isend(&send_buffers[i][0], message_set[i].size(), MPI_Msg, i, tag, MPI_COMM_WORLD, send_requests[i]);
            tag += 1;
            message_set[i].clear(); //clear data
        }
    }
    num_sends += 1;
}

int HDAStar::receive_message_set()
{
    int flag, size;
    MPI_Status status;
    MPI_Iprobe(MPI_ANY_SOURCE, MPI_ANY_TAG, MPI_COMM_WORLD, &flag, &status);
    int buf_size=0;
    
    while(flag)
    {
        MPI_Get_count(&status, MPI_Msg, &size);
        //receive_buffer   
        MPI_Recv(recv_buffer+buf_size, size, MPI_Msg, status.MPI_SOURCE, status.MPI_TAG, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
        buf_size += size;
        MPI_Iprobe(MPI_ANY_SOURCE, MPI_ANY_TAG, MPI_COMM_WORLD, &flag, &status);
    }
    return buf_size;
}

void HDAStar::add_msgs_to_open_list(int num_msgs){
    msg msg_;

    for(int i = 0; i < num_msgs; i++)
    {
        msg_ = recv_buffer[i];
        auto next = msg_.node;
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

    }
    message_set[pid].clear();

}

// find path by time-space A* search
// Returns a bounded-suboptimal path that satisfies the constraints of the give node  while
// minimizing the number of internal conflicts (that is conflicts with known_paths for other agents found so far).
// lowerbound is an underestimation of the length of the path in order to speed up the search.
Path HDAStar::findSuboptimalPath()
{
    MPI_Comm_rank(MPI_COMM_WORLD, &pid);
    MPI_Comm_size(MPI_COMM_WORLD, &nproc);

	MPI_Request dst_req, barrier_req;

    Path path;
    num_expanded = 0;
    num_generated = 0;

    // generate start and add it to the OPEN & FOCAL list
    auto start = new AStarNode(start_location, 0, my_heuristic[start_location], nullptr, 0, 0);
    if (hash(start) == pid)
    {
        pushNode(start);
        allNodes_table.insert(start);
    }

    auto goal_dummy = AStarNode(goal_location, 0, my_heuristic[goal_location], nullptr, 0, 0);
    int dst_pid = hash(&goal_dummy);
    int dst_rcv, dst_flag, barrier_flag = 0;
    if (dst_pid != pid)
    {
        //in this broadcast we will wait for information about whether the destination has been found
        MPI_Ibcast(&dst_rcv, 1, MPI_INT, dst_pid, MPI_COMM_WORLD, &dst_req);
    }

    //register mpi data type
    create_msg_mpi_datatype();
    MPI_Barrier(MPI_COMM_WORLD);

    //receive any message from anywhere 
    message_set.resize(nproc);
    send_buffers.resize(nproc);
    send_requests.resize(nproc, nullptr);

    dst_found = false;

    while (true) {
        // Step 2: process current open list and populate message set
        if (!open_list.empty() && (!in_barrier_mode)){
            auto* curr = popNode();
            assert(curr->location >= 0);
            // check if the popped node is a goal
            if (curr->location == goal_location) // arrive at the goal location
            {
                // the first to find goal might not be optimal
                if (!dst_found)
                {
                    dst_found = true;
                    dst_rcv = curr->getFVal();
                    // broadcast the cost of this path to all processors
		            MPI_Ibcast(&dst_rcv, 1, MPI_INT, dst_pid, MPI_COMM_WORLD, &dst_req);
                    updatePath(curr, path);
                }
                continue;
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

                message_set[hash(next)].push_back(create_msg(next));
            }  
        } else {
            // open list is empty
            if (!dst_found)
            {
                if(dst_pid != pid)
                {
                    MPI_Test(&dst_req, &dst_flag, MPI_STATUS_IGNORE);
                    if(dst_flag){
                        dst_found = true;
		            }
		        }
            } else {
                if(!in_barrier_mode)
                {
                    MPI_Ibarrier(MPI_COMM_WORLD, &barrier_req);
                    in_barrier_mode = true;
                } else {
                    MPI_Test(&barrier_req, &barrier_flag, MPI_STATUS_IGNORE);
                    if (barrier_req)
                    {
                        int to_send = open_list.size(), to_recv = 0;
                        if (to_send)
                        {
                            auto node = open_list.top();
                            if (node->getFVal() > dst_rcv)
                                to_send = 0;
                        }

                        MPI_Allreduce(&to_send, &to_recv, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);
                        if (to_recv == 0)
                        {
                            std::cout << "Program Finished executing.. " << std::endl;
                            // updatePath(curr, path);
                            break;
                        } else {
                            in_barrier_mode = false;
                        }

                    }
                }
            }
        }

        //step 3: send messages
        send_message_set();

        //step 4: receive message set
        int num_msgs = receive_message_set();

        add_msgs_to_open_list(num_msgs);
    }

    releaseNodes();
    return path;
}

inline AStarNode* HDAStar::popNode()
{
    auto node = open_list.top(); open_list.pop();
    open_list.erase(node->open_handle);
    node->in_openlist = false;
    num_expanded++;
    return node;
}

inline void HDAStar::pushNode(AStarNode* node)
{
    node->open_handle = open_list.push(node);
    node->in_openlist = true;
    num_generated++;
}


void HDAStar::releaseNodes()
{
    open_list.clear();
    for (auto node: allNodes_table)
        delete node;
    allNodes_table.clear();
}
