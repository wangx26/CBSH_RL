from libcpp cimport string

cdef extern from "../src/common/agent/agent.cpp":
    pass

cdef extern from "../src/common/agent/agent.h":
    cdef cppclass Agent:
        Agent(string) except +
        void SetStart(int)
        void SetGoal(int)