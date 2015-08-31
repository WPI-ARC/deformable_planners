#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "deformable_astar/Astar.h"
#include <time.h>

StateList * AstarSearch(state6D * startState, state6D * goalState, float ParetoWeight, float (*HeuristicFn)(state6D * stateToEval, state6D * goalState, int debug_level), StateList * (*GenerateChildren)(state6D * state, int debug_level), int ctrl)
{
    //Make open set P-Queue
    printf("Building the storage containers for A*...\n");
    //Make the open set PQueue
    PQueue * openSet = AllocatePQueue();
    //Make the open and closed set hash tables
    HashList * open_hashes = AllocateHashList();
    HashList * closed_hashes = AllocateHashList();
    //Make empty return
    StateList * searchPath = NULL;
    //Load the start state into the sets
    PQueuePush(openSet, startState);
    printf("Attempting to populate the hash table...\n");
    fflush(stdout);
    AddToHashList(open_hashes, startState);
    int nodes_explored = 0;
    struct timespec gst, get, st, et;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &gst);
    printf("Starting the search...\n");
    while (openSet->Length > 0)
    {
        int children_added = 0;
        //Get the current time
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &st);
        debug(ctrl, 1, "A* iteration %d...\n",nodes_explored);
        if (ctrl > 3)
        {
            PrintPQueue(openSet);
            //PrintPQueueReversed(openSet);
        }
        if (ctrl > 4 && (nodes_explored % ctrl) == 0)
        {
            printf("Press ENTER to continue...\n");
            getchar();
        }
        state6D * top_node = PQueuePop(openSet);
        //Since we have no guarantee of uniqueness in the PQueue, we check if this node has already been popped
        state6D * already_popped = CheckInHashList(closed_hashes, top_node);
        if (already_popped != NULL)
        {
            //If it's already in the closed list, then we can skip this one
            /*
            if (!CLOSE_ENOUGH(already_popped->VAL, top_node->VAL))
            {
                printf("Popped duplicate has a different VAL - new: %f, current: %f\n", top_node->VAL, already_popped->VAL);
                printf("New: %f|%f|%f|%f|%f|%f DIST: %f COST: %f VAL: %f\n", top_node->XT, top_node->YT, top_node->ZT, top_node->XR, top_node->YR, top_node->ZR, top_node->DIST, top_node->COST, top_node->VAL);
                printf("Current: %f|%f|%f|%f|%f|%f DIST: %f COST: %f VAL: %f\n", already_popped->XT, already_popped->YT, already_popped->ZT, already_popped->XR, already_popped->YR, already_popped->ZR, already_popped->DIST, already_popped->COST, already_popped->VAL);
                if (top_node->VAL < already_popped->VAL)
                {
                    exit(1);
                }
                if (!StatesEqual(already_popped, top_node))
                {
                    printf("Inconsistent states!\n");
                    exit(1);
                }
            }
            */
            continue;
        }
        //Add the popped node to the closed set and remove from open hashes
        RemoveFromHashList(open_hashes, top_node);
        AddToHashList(closed_hashes, top_node);
        nodes_explored++;
        debug(ctrl, 1, "Popped new state with %f|%f|%f [position], %f|%f|%f [orientation]\n", top_node->XT, top_node->YT, top_node->ZT, top_node->XR, top_node->YR, top_node->ZR);
        debug(ctrl, 2, "Checking if popped node is the goal state\n");
        if (StatesEqual(top_node, goalState))
        {
            debug(ctrl, 0, "Planner found a solution in %d iterations with %d total states generated, %d states in the closed list, and %d states in the open list\n", nodes_explored, closed_hashes->elements + open_hashes->elements, closed_hashes->elements, open_hashes->elements);
            searchPath = BuildPath(top_node);
            break;
        }
        else
        {
            debug(ctrl, 2, "Generating children and evaluating them\n");
            StateList * children = GenerateChildren(top_node, ctrl);
            //printf("%d child states generated...\n", children->Length);
            for (int index = 0; index < children->Length; index++)
            {
                state6D * child = GetIndex(children, index);
                debug(ctrl, 3, "Evaluating new state...\n");
                state6D * temp_state = EvaluateState(child, goalState, ParetoWeight, HeuristicFn, ctrl);
                debug(ctrl, 2, "Examining new child with X:%f, Y:%f, Z:%f...\n", child->XT, child->YT, child->ZT);
                if (temp_state != NULL)
                {
                    debug(ctrl, 2, "Examining new child with X:%f, Y:%f, Z:%f...\n", child->XT, child->YT, child->ZT);
                    //Check if it's in the closed list
                    state6D * closed_existing = CheckInHashList(closed_hashes, child);
                    state6D * open_existing = CheckInHashList(open_hashes, child);
                    if (closed_existing != NULL)
                    {
                        //If it's in the closed list, check the costs
                        ;
                    }
                    else if (open_existing != NULL)
                    {
                        //If it's in the open list, check the costs
                        if ((child->VAL) < (open_existing->VAL))
                        {
                            debug(ctrl, 2, "Replacing a node in the open set with a new one with lower costs\n");
                            ReplaceInHashList(open_hashes, open_existing, child);
                            PQueuePush(openSet, child);
                            children_added++;
                        }
                        else
                        {
                            debug(ctrl, 2, "Deleting a worse version of an existing state\n");
                            DestroyState6D(child);
                        }
                    }
                    else
                    {
                        //Add it automatically
                        debug(ctrl, 2, "Adding a new node to the open set\n");
                        AddToHashList(open_hashes, child);
                        PQueuePush(openSet, child);
                        children_added++;
                    }
                }
                else
                {
                    debug(ctrl, 2, "Deleting impossible state\n");
                    DestroyState6D(child);
                }
            }
        }
        //Print how long the A* iteration took
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &et);
        float secs = (float)(et.tv_sec - st.tv_sec);
        secs = secs + (float)(et.tv_nsec - st.tv_nsec) / 1000000000.0;
        debug(ctrl, 1, "%f seconds per iteration to evaluate %d new children\n", secs, children_added);
    }
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &get);
    float secs = (float)(get.tv_sec - gst.tv_sec);
    secs = secs + (float)(get.tv_nsec - gst.tv_nsec) / 1000000000.0;
    printf("A* search completed in %f seconds\n", secs);
    fflush(stdout);
    //Cleanup, cleanup, everybody cleanup...
    DestroyPQueue(openSet);
    DestroyHashList(open_hashes);
    DestroyHashList(closed_hashes);
    return searchPath;
}

StateList * BuildPath(state6D * top_state)
{
    StateList * plannedPath = AllocateStateList();
    state6D * pointer = top_state;
    while (pointer != NULL)
    {
        StatePrepend(plannedPath, pointer);
        pointer = pointer->parent;
    }
    return plannedPath;
}
