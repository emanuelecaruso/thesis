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
json_path=scene_path_dtam+"state.json"


# Opening JSON file
f = open(json_path)

# returns JSON object as
# a dictionary
data = json.load(f)

# Iterating through the json
# list

size=0.004
coll_name="state_coll"
for coll in bpy.data.collections:
    if coll.name==coll_name:
        for obj in coll.objects:
            bpy.data.objects.remove(obj)
        bpy.data.collections.remove(coll)

coll=bpy.data.collections.new(coll_name)
scene.collection.children.link(coll)

dict = list(data['cameras'].values())[0]
dict2 = list(dict.values())
i=0
vertices = []
edges = []
faces = []

count =0
for value in dict2:
    
    position=value['position']
    depth_var=value['depth_var']
    level=value['level']
    
    if True:
    #if depth_var<0.1:
        len=size*(pow(2,level))
        
        vertices.append( (position[0]+len,position[1]+len,position[2]+len) ) #rtf
        vertices.append( (position[0]+len,position[1]+len,position[2]-len) ) #rtc
        vertices.append( (position[0]+len,position[1]-len,position[2]+len) ) #rdf
        vertices.append( (position[0]+len,position[1]-len,position[2]-len) ) #rdc
        vertices.append( (position[0]-len,position[1]+len,position[2]+len) ) #ltf
        vertices.append( (position[0]-len,position[1]+len,position[2]-len) ) #ltc
        vertices.append( (position[0]-len,position[1]-len,position[2]+len) ) #lcf
        vertices.append( (position[0]-len,position[1]-len,position[2]-len) ) #lcc
        
        faces.append( (count+1,count+2,count+3) ) #r
        faces.append( (count,count+1,count+2) ) #r
        faces.append( (count+5,count+6,count+7) ) #l
        faces.append( (count+4,count+5,count+6) ) #l
        faces.append( (count+1,count+4,count+5) ) #t
        faces.append( (count,count+1,count+4) ) #t
        faces.append( (count+3,count+6,count+7) ) #d
        faces.append( (count+2,count+3,count+6) ) #d
        faces.append( (count+2,count+4,count+6) ) #f
        faces.append( (count,count+2,count+4) ) #f
        faces.append( (count+3,count+5,count+7) ) #c
        faces.append( (count+1,count+3,count+5) ) #c

    
    count+=8

        
        
mesh = bpy.data.meshes.new("mymesh")
obj = bpy.data.objects.new("myobj", mesh)
bpy.context.scene.collection.objects.link(obj)
mesh.from_pydata(vertices, edges, faces)

# Closing file
f.close()
