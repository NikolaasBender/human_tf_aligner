
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
# from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

trans = TransformStamped()

pygame.init()

# open a new window
size = (700, 500)
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


# get the information 
bag = rosbag.Bag('test.bag')
images = [ msg for _, msg, _ in bag.read_messages(topics=['images'])]
velodyne = [ msg for _, msg, _ in bag.read_messages(topics=['velodyne'])]
bag.close()

def ret():
    return True

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

while go:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            go = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            pos = pygame.mouse.get_pos()
            for s in slides:
                if s.button_rect.collidepoint(pos):
                    s.hit = True
            if forward_button.rect.collidepoint(pos):
                index += 1
            if back_button.rect.collidepoint(pos):
                index -= 1
        elif event.type == pygame.MOUSEBUTTONUP:
            for s in slides:
                s.hit = False

    img = images[index]
    pc = velodyne[index]

    for s in slides:
        if s.hit:
            s.move()

    # JAM INPUTS HERE
    tf_update(x.get(), y.get(), z.get(), roll.get(), pitch.get(), yaw.get())

    # put image here
    screen.fill(WHITE)
    screen.blit(vizualize(pc, img), (0,0))

    # display
    pygame.display.flip()

    # 60fps
    clock.tick(60)

pygame.quit()