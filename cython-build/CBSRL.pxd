from libcpp cimport bool

cdef extern from "../src/main.cpp":
    cdef cppclass CBSRL:
        CBSRL() except +
        int init()
        int Reset()
        int Step(int a, int t)
        bool isDone()
        int FinalReward()
        int* GetState()
        int GetMapHeight()
        int GetMapWidth()
        int GetAgentNum()