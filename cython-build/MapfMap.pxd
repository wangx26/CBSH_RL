cdef extern from "../src/common/mapf_map/mapf_map.cpp":
    pass

cdef extern from "../src/common/mapf_map/mapf_map.h":
    cdef cppclass Map:
        Map() except +
        void LoadFileMap()
        int GetWidth()
        int GetHeight()