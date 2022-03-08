import bpy
import os
import json
import shutil
import numpy as np
import math
import sys
from shutil import copy

scene=bpy.context.scene
scene_name=scene.name
render=scene.render

cwd = os.getcwd()

scene_path_dtam=cwd+"/../prova_dr/dataset/"+scene_name+"/"
json_path=scene_path_dtam+"state_cameras.json"


# Opening JSON file
f = open(json_path)

# returns JSON object as
# a dictionary
data = json.load(f)

# Iterating through the json
# list

coll_name="state_cameras_coll"
for coll in bpy.data.collections:
    if coll.name==coll_name:
        for obj in coll.objects:
            bpy.data.objects.remove(obj)
        bpy.data.collections.remove(coll)

coll=bpy.data.collections.new(coll_name)
scene.collection.children.link(coll)

scl=0.03


cam_params = (data['cam_parameters'])
print(cam_params)

lens= cam_params['lens']
width= cam_params['width']
max_depth= cam_params['max_depth']
min_depth= cam_params['min_depth']
    
camera_data = bpy.data.cameras.new('cam_parameters')
camera_data.lens=lens
camera_data.sensor_width=width
camera_data.clip_start=min_depth
camera_data.clip_end=max_depth

cams = list(data['cameras'])

print()
for cam_name in cams:
    print(cam_name)
    pose = list(data['cameras'][cam_name].values())[0]
    #print(pose)
    
    
    camera_object = bpy.data.objects.new(cam_name+'_pred', camera_data)
    bpy.context.scene.collection.objects.link(camera_object)
    print(camera_object.scale)
    camera_object.scale=(scl,scl,scl);
    camera_object.location=(pose[0],pose[1],pose[2])
    camera_object.rotation_mode='XYZ'
    camera_object.rotation_euler.x=pose[3]
    camera_object.rotation_euler.y=pose[4]
    camera_object.rotation_euler.z=pose[5]
    # dict2 = list(camera.values())
    # for value in dict2:
    #     pose = value['pose'];
#
#
# mesh = bpy.data.meshes.new("mymesh")
# obj = bpy.data.objects.new("myobj", mesh)
# bpy.context.scene.collection.objects.link(obj)
# mesh.from_pydata(vertices, edges, faces)

# Closing file
f.close()
