import mpl_interactions.ipyplot as iplt
import matplotlib.pyplot as plt
import numpy as np
import cv2
import os
import time

def lid_pre_process(xyz):
    o =  0 
    if np.size(xyz,1) != 3:
        xyz = np.transpose(xyz)
        
    if np.size(xyz,1) != 3:
        xyz = np.transpose(np.array([xyz[:,0],xyz[:,1],xyz[:,2]]))

    #lidar_pc = np.transpose(lidar_pc[0:3,:])
    
    #lidar_pc[:,o] = -lidar_pc[:,o] 
    #lidar_pc = np.transpose(np.array([lidar_pc[:,1],lidar_pc[:,0],lidar_pc[:,2]]))
    return xyz

# Project lidar points into camera coordinates
def lidar2cam(lidar_pc,rvec,tvec,intrinsics):
    
    dist_coeffs = np.zeros((4,1)) # Assuming no lens distortion

    lidar_px, _ = cv2.projectPoints(lidar_pc, rvec, tvec, intrinsics, dist_coeffs)
    lidar_px = np.squeeze(lidar_px)
    return lidar_px 

path = '/home/nick/catkin_ws/smartbases/bobby_dat'

# get the information 
images = [ cv2.imread(path + "/Images/" + filename) for filename in os.listdir(path + "/Images/")]

leftlid = [ lid_pre_process(np.loadtxt(path + "/LftLidar/" + filename, delimiter=',')) for filename in os.listdir(path + "/LftLidar/") if ".txt" in filename]
rightlid = [ lid_pre_process(np.loadtxt(path + "/RgtLidar/" + filename, delimiter=',')) for filename in os.listdir(path + "/RgtLidar/") if ".txt" in filename]


focal_length = [606.1387,603.1641]
    
center = [959.0102,599.6154]

intrinsics = np.array([[focal_length[0], 0, center[0]],
                       [0, focal_length[1], center[1]],
                       [0, 0, 1]], dtype = "double"
                       ) 

X = np.linspace(0, 100)
Y = np.linspace(0, 100)
Z = np.linspace(0, 100)
Roll = np.linspace(0, 100)
Pitch = np.linspace(0, 100)
Yaw = np.linspace(0, 100)


tvec = np.array((3, 1))
rvec = np.array((3, 1))

index = 0

img = images[index]
pc = leftlid[index]

lidar_points = []


def call(t_x, t_y, t_z, r, p, ya):
    # print(pc.shape)
    tvec = np.array([[float(t_x)], [float(t_y)], [float(t_z)]])
    rvec = np.array([[float(r)], [float(p)], [float(ya)]])
    lidar_pic = lidar2cam(pc, rvec, tvec, intrinsics)
    print(lidar_pic[:,1].squeeze().shape, lidar_pic[:,0].squeeze().shape)
    # lid_small = np.delete(lidar_pic, 2, 1)
    print(lidar_pic.shape)
    # return lidar_pic
    # lidar_points = lidar_pic
    # print(lidar_points.shape)
    # return [lidar_pic[:,1], lidar_pic[:,0]]
    return lidar_pic[:,1]

def call_y(x, t_x, t_y, t_z, r, p, ya):
    # print(pc.shape)
    tvec = np.array([[float(t_x)], [float(t_y)], [float(t_z)]])
    rvec = np.array([[float(r)], [float(p)], [float(ya)]])
    lidar_pic = lidar2cam(pc, rvec, tvec, intrinsics)
    print(lidar_pic[:,1].squeeze().shape, lidar_pic[:,0].squeeze().shape)
    # lid_small = np.delete(lidar_pic, 2, 1)
    print(lidar_pic.shape)
    # return lidar_pic
    # lidar_points = lidar_pic
    # print(lidar_points.shape)
    return lidar_pic[:,0]



# lidar_pic = lidar2cam(pc, rvec, tvec, intrinsics)
fig, ax = plt.subplots()
# ax.imshow(img)
# plt.scatter(lidar_picure[:,1], lidar_picure[:,0],s=1)
# plt.scatter(call(0, 0, 0, 0, 0, 0))

iplt.scatter(call, call_y, t_x=(-1, 1), t_y=(-1, 1), t_z=(0, 4), r=(-2, 2), p=(-2, 2), ya=(-2, 2))
# iplt.scatter(x, f2, controls=controls, label="f2")

# plt.show()
