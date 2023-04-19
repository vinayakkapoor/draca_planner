from temp import Temp

def fun1():
    t = Temp(1,2)
    t.printA()

def fun2():
    t = Temp(3,4)
    t._a = 3
    t.printA()
    fun1()

fun2()