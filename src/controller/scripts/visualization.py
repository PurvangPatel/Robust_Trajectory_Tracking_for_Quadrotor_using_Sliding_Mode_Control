#!/usr/bin/env python3
import os
import matplotlib.pyplot as plt
import pickle
from mpl_toolkits.mplot3d import Axes3D

def visualization(x_series, y_series, z_series):
    # load csv file and plot trajectory
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    # Data for a three-dimensional line
    ax.plot3D(x_series, y_series, z_series, 'blue')    
    ax.plot3D([0, 0, 1, 1, 0, 0], [0, 0, 0, 1, 1, 0], [0, 1, 1, 1, 1, 1], 'green')
    ax.legend(['Actual','Desired'])
    plt.xlim(-0.5, 1.5)
    plt.ylim(-0.5, 1.5)
    plt.minorticks_on()
    plt.grid(which='both')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.savefig('/home/purvang/Robust_Trajectory_Tracking_for_Quadrotor_using_Sliding_Mode_Control/Media/trajectory.png', dpi = 300)
    plt.show()


if __name__ == '__main__':
    file = open(os.getcwd() + "/src/controller/scripts/log.pkl",'rb') 
    t_series, x_series, y_series, z_series = pickle.load(file)
    file.close()
    visualization(x_series, y_series, z_series)