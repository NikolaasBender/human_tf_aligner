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

    cp = np.cos(rvec[0])
    sp = np.sin(rvec[0])
    cq = np.cos(rvec[1])
    sq = np.sin(rvec[1])
    cr = np.cos(rvec[2])
    sr = np.sin(rvec[2])

    roll = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])

    pitch = np.array([[cq, 0, sq], [0, 1, 0], [-sq, 0, cq]])

    yaw = np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]])

    rmtx = yaw @ pitch @ roll

    print(rmtx)

    R, _ = cv2.Rodrigues(np.float32(rmtx))

    lidar_px, _ = cv2.projectPoints(lidar_pc, R, tvec, intrinsics, dist_coeffs)
    lidar_px = np.squeeze(lidar_px)
    return lidar_px 

path = '/home/nick/catkin_ws/smartbases/bobby_dat'

# get the information 
images = [ cv2.cvtColor(cv2.imread(path + "/Images/" + filename), cv2.COLOR_BGR2RGB) for filename in os.listdir(path + "/Images/")]

leftlid = [ lid_pre_process(np.loadtxt(path + "/LftLidar/" + filename, delimiter=',')) for filename in os.listdir(path + "/LftLidar/") if ".txt" in filename]
rightlid = [ lid_pre_process(np.loadtxt(path + "/RgtLidar/" + filename, delimiter=',')) for filename in os.listdir(path + "/RgtLidar/") if ".txt" in filename]


focal_length = [606.1387,603.1641]
    
center = [959.0102,599.6154]

intrinsics = np.array([[focal_length[0], 0, center[0]],
                       [0, focal_length[1], center[1]],
                       [0, 0, 1]], dtype = "double"
                       ) 


index = 3

img = images[index]
pc = leftlid[index]

lidar_points = []

def punchin(img, lidar):
    height = img.shape[0]
    width = img.shape[1]
    trimmed = []
    boundry = 500
    for point in lidar:
        if point[0] < height + boundry and point[1] < width + boundry and point[0] > 0 - boundry and point[1] > 0 - boundry:
            trimmed.append(point)
    return np.array(trimmed)


def call(save, t_x=0.45911911, t_y=-0.43532131, t_z=3.22640716, r=-0.87376982, p=-1.02724177, ya=1.32610297):
    # print(pc.shape)
    tvec = np.array([[float(t_x)], [float(t_y)], [float(t_z)]])
    rvec = np.array([[float(r)], [float(p)], [float(ya)]])
    lidar_pic = lidar2cam(pc, rvec, tvec, intrinsics)
    return punchin(img, lidar_pic)[:,1]

def call_y(x, save, t_x=0.45911911, t_y=-0.43532131, t_z=3.22640716, r=-0.87376982, p=-1.02724177, ya=1.32610297):
    # print(pc.shape)
    tvec = np.array([[float(t_x)], [float(t_y)], [float(t_z)]])
    rvec = np.array([[float(r)], [float(p)], [float(ya)]])
    lidar_pic = lidar2cam(pc, rvec, tvec, intrinsics)
    if save > 1:
        with open("save.txt", 'w+') as file:
            file.write("tx, ty, tz, r, p, y \n{},{},{},{},{},{}".format(t_x, t_y, t_z, r, p, ya))
    return punchin(img, lidar_pic)[:,0]



# lidar_pic = lidar2cam(pc, rvec, tvec, intrinsics)
fig, ax = plt.subplots()
ax.imshow(img)
# plt.scatter(lidar_picure[:,1], lidar_picure[:,0],s=1)
# plt.scatter(call(0, 0, 0, 0, 0, 0))

controls = iplt.scatter(call, call_y, t_x=(-1, 1, 1000), t_y=(-1, 1, 1000), t_z=(-1, 1, 1000), r=(-2*np.pi, 2*np.pi, 1000), p=(-2*np.pi, 2*np.pi, 1000), ya=(-2*np.pi, 2*np.pi, 1000), save=(0, 2, 2), s=1)
# iplt.scatter(x, f2, controls=controls, label="f2")

plt.show()