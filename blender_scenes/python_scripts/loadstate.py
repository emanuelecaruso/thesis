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

cols=640
rows=480

for col in range( cols ):
    for row in range( rows ):

        ul=row+col*rows
        
        value=dict2[ul]
        color=value['color']
        position=value['position']
        valid=value['valid']
        
        vertices.append( (position[0],position[1],position[2]) )
        
        if (col==cols-1 or row==rows-1 or not valid):
            continue
        
        ur=row+(col+1)*rows
        dl=row+1+col*rows
        dr=row+1+(col+1)*rows
        
        if dict2[ur]['valid'] and dict2[dl]['valid']:
            faces.append( (ul,ur,dl) )
            if dict2[dr]['valid']:
                faces.append( (ur,dr,dl) )
        
        
mesh = bpy.data.meshes.new("mymesh")
obj = bpy.data.objects.new("myobj", mesh)
bpy.context.scene.collection.objects.link(obj)
mesh.from_pydata(vertices, edges, faces)

# Closing file
f.close()
