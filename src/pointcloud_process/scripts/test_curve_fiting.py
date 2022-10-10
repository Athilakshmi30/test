#!/usr/bin/env python
'''
import matplotlib.pyplot as plt
import random

fig = plt.figure(figsize=(12, 12))
ax = fig.add_subplot(projection='3d')

sequence_containing_x_vals = list(range(0, 3))
sequence_containing_y_vals = list(range(0, 3))
#sequence_containing_z_vals = list(range(0, 100))

random.shuffle(sequence_containing_x_vals)
random.shuffle(sequence_containing_y_vals)
#random.shuffle(sequence_containing_z_vals)

plt.scatter(sequence_containing_x_vals, sequence_containing_y_vals)
plt.show()


# curve fit function
def func1(t,a,b):
	return a*t+b
def func2(t,a,b,c):
	return a*pow(t,2)+b*t+c
def func3(t,a,b,c,d):
	return a*pow(t,3)+b*pow(t,2)+c*t+d
def func4(t,a,b,c,d,e):
	return a*pow(t,4)+b*pow(t,3)+c*pow(t,2)+d*t+e
def func5(t,a,b,c,d,e,f):
	return a*pow(t,5)+b*pow(t,4)+c*pow(t,3)+d*pow(t,2)+e*t+f
def func6(t,a,b,c,d,e,f,g):
	return a*pow(t,6)+b*pow(t,5)+c*pow(t,4)+d*pow(t,3)+e*pow(t,2)+f*t+g
def func7(t,a,b,c,d,e,f,g,h):
	return a*pow(t,7)+b*pow(t,6)+c*pow(t,5)+d*pow(t,4)+e*pow(t,3)+f*pow(t,2)+g*t+h
def func8(t,a,b,c,d,e,f,g,h,i):
	return a*pow(t,8)+b*pow(t,7)+c*pow(t,6)+d*pow(t,5)+e*pow(t,4)+f*pow(t,3)+g*pow(t,2)+h*t+i
def func8(t,a,b,c,d,e,f,g,h,i,j):
	return a*pow(t,9)+b*pow(t,8)+c*pow(t,7)+d*pow(t,6)+e*pow(t,5)+f*pow(t,4)+g*pow(t,3)+h*pow(t,2)+i*t+j

# reading thermodynamic data file
def read_file():
	temperature = []
	cp = []
	for line in open(\'data\',\'r\'):
		values = line.split(\',\')
		temperature.append(float(values[0]))
		cp.append(float(values[1]))

	return [temperature,cp]

# main program
temperature,cp = read_file()
popt,pcov = curve_fit(func1,temperature,cp)


a, b, c = np.polyfit(X_data, Y_data, 2)
fit_equation = lambda x: a * x ** 2 + b * x + c
'''

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import random
#X_data = [1, 2, 3, 4, 5, 6,4]
#Y_data = [2, 4, 5, 6, 7, 5,3]

f = open("test.txt", "r")
lines =f.readline() 

X_data=[]
Y_data=[]
#Z_data=[]
for points in lines:
    print("points",points)
    for pt in points:
        if pt[0]==0:
            continue
        X_data.append(pt[0])
        Y_data.append(pt[1])
    #Z_data.append(point[2])

#X_data = list(range(0, 5))
#Y_data = list(range(0, 5))
#random.shuffle(X_data)
#random.shuffle(Y_data)

'''
#2deg
a, b, c, d = np.polyfit(X_data, Y_data, 3)
fit_equation = lambda x: a * x ** 3 + b * x **2+ c*x+d
'''
'''
#3deg
a, b, c, d = np.polyfit(X_data, Y_data, 3)
fit_equation = lambda x: a * x ** 3 + b * x **2+ c*x+d
'''
'''
#4deg
a, b, c, d,e = np.polyfit(X_data, Y_data, 4)
fit_equation = lambda x: a * x ** 4 + b * x **3+ c*x**2+d*x+e
'''

#5deg
a, b, c, d, e, f = np.polyfit(X_data, Y_data, 5)
fit_equation = lambda x: a * x ** 5 + b * x **4+ c*x**3+d*x**2+e*x+f


def plot_fit(X, Y, f):
    X_fit = np.linspace(min(X), max(X), 1000)
    Y_fit = f(X_fit)

    fig, ax1 = plt.subplots()
    #print(X_fit)
    #print(Y_fit)


    ax1.plot(X_fit, Y_fit, color='r', alpha=0.5, label='Polynomial fit')
    ax1.scatter(X, Y, s=4, color='b', label='Data points')
    ax1.set_title('Polynomial fit example')
    ax1.legend()
    plt.show()
    
plot_fit(X_data, Y_data, fit_equation)
