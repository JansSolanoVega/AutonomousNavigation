from matplotlib import pyplot as plt
from matplotlib import animation 
from matplotlib import style 
import serial 

fig,ax1=plt.subplots(figsize=(12,6))
ax1.set_xlabel("X")
ax1.set_ylabel("Y")
ax1.set_title("CAR'S PATH")
ser = serial.Serial('COM3', 115200) 
ax1.plot(50, 50, "bo-")
def plotea (i):
    xx=[]
    yy=[]
    for i in range(50):
        data=ser.readline()
        data=data.splitlines()[0]
        data=data.decode().split(",")
        xx.append(float(data[0]))
        yy.append(float(data[1]))
        print(data[0],data[1])
    ax1.set_ylim([-30, 60])
    ax1.set_xlim([-30,60])

    try:  # Nos permite comprobar si hay un error al ejecutar la siguiente instrucción.
        ax1.plot(xx,yy, "r--")  
    except: 
        pass

ani = animation.FuncAnimation(fig, plotea, interval = 1) #Creamos animación para que se ejecute la función plotea con un intervalo de 1ms.

plt.show() #Muestra la gráfica.
