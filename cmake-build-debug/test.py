import cbsrl

a = cbsrl.CBSHRL()
print(a.getstate())
a.isdone()
a.step(0, 3)
print(a.getstate())