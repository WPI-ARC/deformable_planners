#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "deformable_astar/SearchTools.h"

state6D * NewState6D(state6D * parent, dVoxelArray * RefShape, dVoxelArray * Environment, float XT, float YT, float ZT, float XR, float YR, float ZR, float distDelta)
{
    state6D * new_state = (state6D*) malloc(sizeof(state6D));
    if (parent != NULL)
    {
        new_state->XT = XT;
        new_state->YT = YT;
        new_state->ZT = ZT;
        new_state->XR = XR;
        new_state->YR = YR;
        new_state->ZR = ZR;
        new_state->parent = parent;
        new_state->RefShape = RefShape;
        new_state->Environment = Environment;
        new_state->COST = parent->COST;
        new_state->DIST = parent->DIST + distDelta;
        new_state->VAL = -1.0;
    }
    else
    {
        new_state->XT = XT;
        new_state->YT = YT;
        new_state->ZT = ZT;
        new_state->XR = XR;
        new_state->YR = YR;
        new_state->ZR = ZR;
        new_state->parent = NULL;
        new_state->RefShape = RefShape;
        new_state->Environment = Environment;
        new_state->COST = 0;
        new_state->DIST = distDelta;
        new_state->VAL = -1.0;
    }
    new_state->key = StateToKeyStr(new_state);
    return new_state;
}

int DestroyState6D(state6D * state)
{
    state->parent = NULL;
    state->RefShape = NULL;
    state->Environment = NULL;
    free(state->key);
    free(state);
    return 0;
}

int StatesEqual(state6D * state1, state6D * state2)
{
    //printf("Comparing %f|%f|%f,%f|%f|%f to %f|%f|%f,%f|%f|%f\n", state1->XT, state1->YT, state1->ZT, state1->XR, state1->YR, state1->ZR, state2->XT, state2->YT, state2->ZT, state2->XR, state2->YR, state2->ZR);
    if (CLOSE_ENOUGH(state1->XT, state2->XT) && CLOSE_ENOUGH(state1->YT, state2->YT) && CLOSE_ENOUGH(state1->ZT, state2->ZT) && CLOSE_ENOUGH(state1->XR, state2->XR) && CLOSE_ENOUGH(state1->YR, state2->YR) && CLOSE_ENOUGH(state1->ZR, state2->ZR))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

char * StateToKeyStr(state6D * state)
{
    size_t key_length = 128;
    char * key = (char*)malloc(sizeof(char) * key_length);
    int length = snprintf(key, key_length, "%.4f|%.4f|%.4f|%.4f|%.4f|%.4f", state->XT, state->XR, state->YT, state->YR, state->ZT, state->ZR);
    if (length > 0 && length < (int)key_length)
    {
        return key;
    }
    else
    {
        printf("Key length exceded key buffer!\n");
        fflush(stdout);
        return NULL;
    }
}

Node * NewNode(Node * prev, Node * next, state6D * data)
{
    Node * new_node = (Node*) malloc(sizeof(Node));
    new_node->prev_node = prev;
    new_node->next_node = next;
    new_node->state = data;
    return new_node;
}

int DestroyNode(Node * nodeToDelete)
{
    nodeToDelete->prev_node = NULL;
    nodeToDelete->next_node = NULL;
    nodeToDelete->state = NULL;
    free(nodeToDelete);
    return 0;
}

StateList * AllocateStateList()
{
    StateList * states = (StateList*) malloc(sizeof(StateList));
    states->Length = 0;
    states->HeadNode = NULL;
    states->TailNode = NULL;
    return states;
}

int DestroyStateList(StateList * states)
{
    states->Length = 0;
    states->HeadNode = NULL;
    states->TailNode = NULL;
    free(states);
    return 0;
}

int StateAppend(StateList * states, state6D * new_state)
{
    Node * new_node = NewNode(NULL, NULL, new_state);
    if (states->Length > 0)
    {
        states->TailNode->next_node = new_node;
        new_node->prev_node = states->TailNode;
        states->TailNode = new_node;
        states->Length++;
        return 0;
    }
    else
    {
        states->HeadNode = new_node;
        states->TailNode = new_node;
        states->Length++;
        return 0;
    }
}

int StatePrepend(StateList * states, state6D * new_state)
{
    Node * new_node = NewNode(NULL, NULL, new_state);
    if (states->Length > 0)
    {
        states->HeadNode->prev_node = new_node;
        new_node->next_node = states->HeadNode;
        states->HeadNode = new_node;
        states->Length++;
        return 0;
    }
    else
    {
        states->HeadNode = new_node;
        states->TailNode = new_node;
        states->Length++;
        return 0;
    }
}

state6D * GetIndex(StateList * states, int index)
{
    if (index >= states->Length)
    {
        return NULL;
    }
    else
    {
        Node * pointer = states->HeadNode;
        int count = 0;
        while (count < index)
        {
            pointer = pointer->next_node;
            count++;
        }
        return pointer->state;
    }
}

int StateContained(StateList * states, state6D * new_state)
{
    if (states->Length == 0)
    {
        return -1;
    }
    else
    {
        Node * pointer = states->HeadNode;
        while (pointer != NULL)
        {
            if (StatesEqual(pointer->state, new_state) == 1)
            {
                return 1;
            }
            else
            {
                pointer = pointer->next_node;
            }
        }
        return 0;
    }
}

PQueue * AllocatePQueue()
{
    PQueue * queue = (PQueue*) malloc(sizeof(PQueue));
    queue->Length = 0;
    queue->HeadNode = NULL;
    queue->TailNode = NULL;
    return queue;
}

int DestroyPQueue(PQueue * queue)
{
    queue->Length = 0;
    queue->HeadNode = NULL;
    queue->TailNode = NULL;
    free(queue);
    return 0;
}

int PQueuePush(PQueue * queue, state6D * new_state)
{
    if (queue == NULL)
    {
        printf("***CANNOT PUSH TO A NULL QUEUE***\n");
        return -1;
    }
    //printf("Inserting into a PQueue of length %d...\n", queue->Length);
    Node * new_node = NewNode(NULL, NULL, new_state);
    if (queue->Length == 0)
    {
        //printf("Pushing the first node...\n");
        queue->HeadNode = new_node;
        queue->TailNode = new_node;
        queue->Length++;
        return 0;
    }
    else
    {
        // Run through some special cases first
        // Check if the new state is better than the first stored
        if (new_node->state->VAL <= queue->HeadNode->state->VAL)
        {
            //printf("Inserting at the head of the queue - new VAL: %f, head VAL: %f\n", new_node->state->VAL, queue->HeadNode->state->VAL);
            new_node->next_node = queue->HeadNode;
            queue->HeadNode->prev_node = new_node;
            queue->HeadNode = new_node;
            queue->Length++;
            return 0;
        }
        // Check if the new state is worse than all stored
        if (new_node->state->VAL >= queue->TailNode->state->VAL)
        {
            //printf("Inserting at the tail of the queue - new VAL: %f, tail VAL: %f\n", new_node->state->VAL, queue->TailNode->state->VAL);
            new_node->prev_node = queue->TailNode;
            queue->TailNode->next_node = new_node;
            queue->TailNode = new_node;
            queue->Length++;
            return (queue->Length - 1);
        }
        // Now, handle the general case
        //printf("Inserting a node in ascending order...\n");
        Node * pointer = queue->HeadNode;
        int index = 0;
        while (pointer != NULL)
        {
            if (new_node->state->VAL <= pointer->state->VAL)
            {
                //Insert ahead of pointer
                //printf("Inserting ahead of pointer - new VAL: %f, pointer VAL: %f\n", new_node->state->VAL, pointer->state->VAL);
                new_node->next_node = pointer;
                new_node->prev_node = pointer->prev_node;
                new_node->prev_node->next_node = new_node;
                pointer->prev_node = new_node;
                queue->Length++;
                return index;
            }
            else if (pointer->next_node == NULL)
            {
                //We've reached the end of the list, add to the end
                //printf("Inserting at the tail of the queue - new VAL: %f, tail VAL: %f\n", new_node->state->VAL, queue->TailNode->state->VAL);
                new_node->prev_node = queue->TailNode;
                queue->TailNode->next_node = new_node;
                queue->TailNode = new_node;
                queue->Length++;
                return index;
            }
            else if (new_node->state->VAL > pointer->state->VAL)
            {
                //We're still looking
                //printf("Moving through the queue - new VAL: %f, pointer VAL: %f\n", new_node->state->VAL, pointer->state->VAL);
                pointer = pointer->next_node;
                index++;
            }
            else
            {
                printf("*** WE REALLY SHOULD NOT BE HERE ***\n");
            }
        }
        return -1;
    }
}



state6D * PQueuePop(PQueue * queue)
{
    if (queue->Length == 0)
    {
        return NULL;
    }
    else
    {
        if (queue->Length == 1)
        {
            state6D * top_state = queue->HeadNode->state;
            Node * top_node = queue->HeadNode;
            queue->HeadNode = NULL;
            queue->TailNode = NULL;
            queue->Length = 0;
            DestroyNode(top_node);
            return top_state;
        }
        else if (queue->Length > 1)
        {
            state6D * top_state = queue->HeadNode->state;
            Node * top_node = queue->HeadNode;
            queue->HeadNode = queue->HeadNode->next_node;
            queue->HeadNode->prev_node = NULL;
            queue->Length--;
            DestroyNode(top_node);
            return top_state;
        }
        else
        {
            printf("PQueue has a negative index count!\n");
            return NULL;
        }
    }
}

void PrintPQueue(PQueue * queue)
{
    Node * pointer = queue->HeadNode;
    int index = 0;
    printf("PQueue contents:\n");
    while (pointer != NULL)
    {
        printf("Element %d: ", index);
        printf("contents: %f|%f|%f|%f|%f|%f DIST: %f COST: %f VAL: %f\n", pointer->state->XT, pointer->state->YT, pointer->state->ZT, pointer->state->XR, pointer->state->YR, pointer->state->ZR, pointer->state->DIST, pointer->state->COST, pointer->state->VAL);
        pointer = pointer->next_node;
        index++;
    }
}

void PrintPQueueReversed(PQueue * queue)
{
    Node * pointer = queue->TailNode;
    int index = queue->Length - 1;
    printf("PQueue contents (reversed):\n");
    while (pointer != NULL)
    {
        printf("Element %d: ", index);
        printf("contents: %f|%f|%f|%f|%f|%f DIST: %f COST: %f VAL: %f\n", pointer->state->XT, pointer->state->YT, pointer->state->ZT, pointer->state->XR, pointer->state->YR, pointer->state->ZR, pointer->state->DIST, pointer->state->COST, pointer->state->VAL);
        pointer = pointer->prev_node;
        index--;
    }
}

HashList * AllocateHashList()
{
    HashList * hashlist = (HashList*) malloc(sizeof(HashList));
    hashlist->elements = 0;
    hashlist->table = NULL;
    return hashlist;
}

int DestroyHashList(HashList * hashlist)
{
    hashlist->elements = 0;
    HASH_CLEAR(hh,hashlist->table);
    free(hashlist);
    return 0;
}

int AddToHashList(HashList * hashlist, state6D * new_state)
{
    HASH_ADD_KEYPTR(hh, hashlist->table, new_state->key, strlen(new_state->key), new_state);
    hashlist->elements++;
    return hashlist->elements;
}

state6D * RemoveFromHashList(HashList * hashlist, state6D * to_remove)
{
    state6D * checked = CheckInHashList(hashlist, to_remove);
    if (checked != NULL)
    {
        HASH_DEL(hashlist->table, checked);
        return checked;
    }
    return NULL;
}

state6D * CheckInHashList(HashList * hashlist, state6D * potential_state)
{
    state6D * current_value;
    HASH_FIND(hh, hashlist->table, potential_state->key, strlen(potential_state->key), current_value);
    return current_value;
}

int ReplaceInHashList(HashList * hashlist, state6D * current_state, state6D * replacement)
{
    state6D * current_value = CheckInHashList(hashlist, current_state);
    if (current_value != NULL)
    {
        //Safety check
        //if (current_value->XT != replacement->XT || current_value->YT != replacement->YT || current_value->ZT != replacement->ZT || current_value->XR != replacement->XR || current_value->YR != replacement->YR || current_value->ZR != replacement->ZR)
        if (!CLOSE_ENOUGH(current_value->XT, replacement->XT) || !CLOSE_ENOUGH(current_value->YT, replacement->YT) || !CLOSE_ENOUGH(current_value->ZT, replacement->ZT) || !CLOSE_ENOUGH(current_value->XR, replacement->XR) || !CLOSE_ENOUGH(current_value->YR, replacement->YR) || !CLOSE_ENOUGH(current_value->ZR, replacement->ZR))
        {
            if (current_value->XT != replacement->XT)
            {
                printf("XT value mismatch\n");
            }
            if (current_value->YT != replacement->YT)
            {
                printf("YT value mismatch\n");
            }
            if (current_value->ZT != replacement->ZT)
            {
                printf("ZT value mismatch\n");
            }
            if (current_value->XR != replacement->XR)
            {
                printf("XR value mismatch\n");
            }
            if (current_value->YR != replacement->YR)
            {
                printf("YR value mismatch\n");
            }
            if (current_value->ZR != replacement->ZR)
            {
                printf("ZR value mismatch\n");
            }
            printf("Stored state does not match replacement!\n");
            printf("Stored key: %s\n", current_state->key);
            printf("Replacement key: %s\n", replacement->key);
            printf("Stored state: %f|%f|%f|%f|%f|%f\n", current_state->XT, current_state->YT, current_state->ZT, current_state->XR, current_state->YR, current_state->ZR);
            printf("Replacement state: %f|%f|%f|%f|%f|%f\n", replacement->XT, replacement->YT, replacement->ZT, replacement->XR, replacement->YR, replacement->ZR);
            fflush(stdout);
            exit(2);
        }
        current_value->COST = replacement->COST;
        current_value->DIST = replacement->DIST;
        current_value->parent = replacement->parent;
        current_value->VAL = replacement->VAL;
        return hashlist->elements;
    }
    return -1;
}
