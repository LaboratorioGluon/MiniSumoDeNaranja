import numpy as np
import itertools
import matplotlib.pyplot as plt

def req( res ):
    if len(res) == 0:
        return 0
    return 1/np.sum(np.reciprocal(res))
    
def binListToDec(l):
    dec = 0
    l.reverse()
    cnt = 0
    for i in l:
        suma= i*(2**cnt)
        dec += suma
        cnt += 1
    return dec
    
RC = 2200
N = 5
def R(n):
    #return (N/n - 1)*RC
    return RC*(2**(N/n-1))
    
_r = [R(n+1) for n in range(N)]
resistors = np.array(_r, dtype=float)[:-1]
#resistors = np.array([0.001,1000,2200,4700], dtype=float)
resistors = np.array([0.001,2200,4700,10000], dtype=float)
lst = list(map(list, itertools.product([0, 1], repeat=N-1)))
print(str(lst))

resultados = []
for i in lst:
    #if lst.index(i) % 2 :
    #    continue
    resloop = np.multiply(resistors, i)
    filterd = [r for r in resloop if r != 0]
    resistoreq = req(filterd)
    switches = "".join(map(str,i))
    decs = binListToDec(i)
    v = 3.3*RC/(resistoreq+RC)
    resultados.append([switches,decs,resistoreq,v])
    print(f"{switches}-{decs}\t{resistoreq:.2f}\t{v:.2f}")

npres = np.array(resultados, dtype=float)



plt.plot(npres[:,1], npres[:,3], 'o')
plt.hlines(3.3,-2,20,'r')
plt.ylim([0, 4])
plt.show()