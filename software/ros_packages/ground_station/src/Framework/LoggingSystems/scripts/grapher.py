from matplotlib import pyplot as plt 
import matplotlib.animation as animation
import time
import asyncio

plt.ion()
async def graph(x,y):
	while True == True:
		plt.show()
		plt.plot(x,y)
		plt.draw()
		plt.pause(1)

x = [5,2,9,4,7]
y = [10,5,8,4,2]

asyncio.run(graph(x,y))

print('hello world')
