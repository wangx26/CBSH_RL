import cbsrl

a = cbsrl.CBSHRL()
while not a.isdone():
    print("step")
    _ = a.stepLorR(1)
a.reset(True)
print(a.getstate())