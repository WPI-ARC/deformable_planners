#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "dVoxelLib.h"
#include "uthash.h"

/**
    Tools for state storage and search
*/

/**
    Storage for an individual 6-dimensional storage state
*/
typedef struct state6D {
    float XT;
    float YT;
    float ZT;
    float XR;
    float YR;
    float ZR;
    float COST;
    float DIST;
    float VAL;
    char * key;
    state6D * parent;
    dVoxelArray * RefShape;
    dVoxelArray * Environment;
    UT_hash_handle hh;
} state6D;

state6D * NewState6D(state6D * parent, dVoxelArray * RefShape, dVoxelArray * Environment, float XT, float YT, float ZT, float XR, float YR, float ZR, float distDelta);

int DestroyState6D(state6D * state);

int StatesEqual(state6D * state1, state6D * state2);

char * StateToKeyStr(state6D * state);

/**
    Basic building block of linked-list type data storage
*/
typedef struct Node {
    Node * prev_node;
    Node * next_node;
    state6D * state;
} Node;

Node * NewNode(Node * prev, Node * next, state6D * data);

int DestroyNode(Node * nodeToDelete);

/**
    Simple doubly-linked list storage with insert at head and insert at tail
*/
typedef struct StateList {
    int Length;
    Node * HeadNode;
    Node * TailNode;
} StateList;

StateList * AllocateStateList();

int DestroyStateList(StateList * states);

int StateAppend(StateList * states, state6D * new_state);

int StatePrepend(StateList * states, state6D * new_state);

state6D * GetIndex(StateList * states, int index);

int StateContained(StateList * states, state6D * new_state);

/**
    Simple priority queue using doubly linked nodes. Inserts new values sorted
*/
typedef struct PQueue {
    int Length;
    Node * HeadNode;
    Node * TailNode;
} PQueue;

PQueue * AllocatePQueue();

int DestroyPQueue(PQueue * queue);

void PrintPQueue(PQueue * queue);

void PrintPQueueReversed(PQueue * queue);

int PQueuePush(PQueue * queue, state6D * new_state);

state6D * PQueuePop(PQueue * queue);

//    Hash-table based state storage (constant-time access)

typedef struct HashList {
    long elements;
    state6D * table;
} HashList;

HashList * AllocateHashList();

int DestroyHashList(HashList * hashlist);

int AddToHashList(HashList * hashlist, state6D * new_state);

state6D * RemoveFromHashList(HashList * hashlist, state6D * to_remove);

state6D * CheckInHashList(HashList * hashlist, state6D * potential_state);

int ReplaceInHashList(HashList * hashlist, state6D * current_state, state6D * replacement);
