import cbsrl

cbsrl_obj = cbsrl.PyCbsrl()
cbsrl_obj.cbsrl_init()
h = cbsrl_obj.cbsrl_getmaph()
w = cbsrl_obj.cbsrl_getmapw()
print(h, w)