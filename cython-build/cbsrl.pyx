# distutils: language = c++

from CBSRL cimport CBSRL
from MapfMap cimport Map
from Agent cimport Agent

cdef class PyCbsrl:
    cdef CBSRL c_cbsrl

    def __c_init__(self):
        self.c_cbsrl = CBSRL()
    def cbsrl_init(self):
        return self.c_cbsrl.init()
    def cbsrl_getmaph(self):
        return self.c_cbsrl.GetMapHeight()
    def cbsrl_getmapw(self):
        return self.c_cbsrl.GetMapWidth()

cdef class PyCBSSearch:
    cdef CBSHSearch*cbs_search

    def __c_init__(self, )
        self.cbs_search = new CBSHSearch()
    def __dealloc__(self):
        del self.cbs_search