rom MapfMap cimport Map
from Agent cimport Agent

cdef extern from "../src/algorithm/CBSH2/CBSHSearch.cpp":
    pass

cdef extern from "../src/algorithm/CBSH2/CBSHSearch.h" namespace "mapf::CBSH":
    cdef cppclass CBSHSearch:
        CBSHSearch(Map, Agent, ) except +