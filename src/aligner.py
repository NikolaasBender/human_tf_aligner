
# Import the pygame library and initialise the game engine
import pygame
import rospy
import math
from geometry_msgs.msg import TransformStamped
import numpy as np
import rosbag
from interface import *
# from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
# from tf.transformations import quaternion_from_euler
import numpy as np
import cv2
import matplotlib.pyplot as plt
# import tf2_ros
# import tf2_py as tf2
from sensor_msgs.msg import PointCloud2
import os
import pylab
# from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_clou
import matplotlib
matplotlib.use("Agg")

import matplotlib.backends.backend_agg as agg

fig = pylab.figure(figsize=[7, 7], # Inches
                   dpi=100,        # 100 dots per inch, so the resulting buffer is 400x400 pixels
                   )
ax = fig.gca()

pygame.init()

# open a new window
size = (1000, 1000)
screen = pygame.display.set_mode(size)
pygame.display.set_caption("transform me")

go = True

clock = pygame.time.Clock()

def tf_update(x, y, z, r, p, yaw):
    trans.transform.translation.x = x
    trans.transform.translation.y = y
    trans.transform.translation.z = z
    # q = quaternion_from_euler(r, p, yaw)
    # trans.transform.rotation.x = q[0]
    # trans.transform.rotation.y = q[1]
    # trans.transform.rotation.z = q[2]
    # trans.transform.rotation.w = q[3]


# combine the transformed lidar and the image to make a new image
# lidar is a full pointcloud2 lidar msg
# the transform should probably be a transform stamped message
def vizualize(lidar, img):
    trans.header.stamp = rospy.Time.now()
    trans.header.frame_id = "CHANGE ME"
    # cloud_out = do_transform_cloud(lidar, trans)
    return True


# this should save the transform to a file
def save_tf(tf):
    with open('tf.txt', 'w') as file:
        file.write(tf)
    return True



# Project lidar points into camera coordinates
def lidar2cam(lidar_pc,rvec,tvec,intrinsics):
    
    dist_coeffs = np.zeros((4,1)) # Assuming no lens distortion

    lidar_px, _ = cv2.projectPoints(lidar_pc, rvec, tvec, intrinsics, dist_coeffs)
    lidar_px = np.squeeze(lidar_px)
    return lidar_px 


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

def combiner(lidar, image):
    rectangle_scale = 5
    color = (0, 0, 0)
    thickness = -1
    height, width, _ = image.shape
    for point in lidar:
        print(point)
        if point[0] <= width and point[1] <= height and point[0] >= 0 and point[1] >= 0:
            start = (int(point[0]) - rectangle_scale, int(point[1]) - rectangle_scale)
            end = (int(point[0]) + rectangle_scale, int(point[1]) + rectangle_scale)
            image = cv2.rectangle(image, start, end, color, thickness)
        else:
            print('cant place point')
    image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return image

path = '/home/nick/catkin_ws/smartbases/bobby_dat'

# get the information 
images = [ cv2.imread(path + "/Images/" + filename) for filename in os.listdir(path + "/Images/")]

leftlid = [ lid_pre_process(np.loadtxt(path + "/LftLidar/" + filename, delimiter=',')) for filename in os.listdir(path + "/LftLidar/") if ".txt" in filename]
rightlid = [ lid_pre_process(np.loadtxt(path + "/RgtLidar/" + filename, delimiter=',')) for filename in os.listdir(path + "/RgtLidar/") if ".txt" in filename]



def ret():
    return True

focal_length = [606.1387,603.1641]
    
center = [959.0102,599.6154]

intrinsics = np.array([[focal_length[0], 0, center[0]],
                       [0, focal_length[1], center[1]],
                       [0, 0, 1]], dtype = "double"
                       ) 

forward_button = Button('next', (100, 100), ret)
back_button = Button('prev', (100, 200), ret)
x = Slider("Pen", 10, 15, 1, 25)
y = Slider("Freq", 1, 3, 0.2, 150)
z = Slider("Jump", 10, 20, 1, 275)
roll = Slider("Size", 200, 200, 20, 400)
pitch = Slider("Focus", 0, 6, 0, 525)
yaw = Slider("Phase", 3.14, 6, 0.3, 650)
slides = [x, y, z, roll, pitch, yaw]

index = 0

print('powered by crayola')

while go:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            go = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            pos = pygame.mouse.get_pos()
            for s in slides:
                if s.button_rect.collidepoint(pos):
                    s.hit = True
            if forward_button.collidepoint(pos):
                index += 1
            if back_button.collidepoint(pos):
                index -= 1
        elif event.type == pygame.MOUSEBUTTONUP:
            for s in slides:
                s.hit = False

    img = images[index]
    pc = leftlid[index]

    for s in slides:
        if s.hit:
            s.move()

    # JAM INPUTS HERE
    tvec = np.array([[float(x.get())], [float(y.get())], [float(z.get())]])
    rvec = np.array([[float(roll.get())], [float(pitch.get())], [float(yaw.get())]])

    # put image here
    screen.fill(WHITE)
    lidar_pic = lidar2cam(pc, rvec, tvec, intrinsics)
    # combined_img = combiner(lidar_pic, img)
    # surf = pygame.surfarray.make_surface(combined_img)

    fig, ax = plt.subplots()
    ax.imshow(img)
    plt.scatter(lidar_pic[:,1], lidar_pic[:,0],s=1)

    canvas = agg.FigureCanvasAgg(fig)
    canv_size = canvas.get_width_height()
    canvas.draw()
    renderer = canvas.get_renderer()
    raw_data = renderer.tostring_rgb()

    surf = pygame.image.fromstring(raw_data, canv_size, "RGB")
    screen.blit(surf, (0,0))

    # display
    pygame.display.flip()

    # 60fps
    clock.tick(60)

pygame.quit()