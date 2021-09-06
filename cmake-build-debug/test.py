import cbsrl

a = cbsrl.CBSHRL()
while not a.isdone():
    print(a.getstate())
    a.stepLorR(1)
a.reset(True)
print(a.getstate())