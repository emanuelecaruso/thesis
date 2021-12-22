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
json_path=scene_path_dtam+scene_name+".json"

rem = False
con = False

if os.path.isdir(scene_path_dtam):
    print("directory "+scene_path_dtam+" already exists")
    while(True):
        print("Enter:")
        print("r to remove existing dataset")
        print("c to continue the existing dataset")
        print("q to quit")
        input1 = input()
        if (input1=="r"):
            rem=True
            break
        elif (input1=="c"):
            con=True
            break
        elif (input1=="q"):
            sys.exit()

if(rem):
    shutil.rmtree(scene_path_dtam, ignore_errors=True)

scene.display_settings.display_device = 'sRGB'


#############################################################################
#		generate json for dtam and render images
#############################################################################

engine=''
argv=sys.argv;
if argv[-2]=='CYCLES':
    engine='CYCLES'
    bpy.context.scene.cycles.samples = int(argv[-1])
else:
    engine='BLENDER_EEVEE'

engine_eevee='BLENDER_EEVEE'

scene.use_nodes = True
tree = scene.node_tree
links = tree.links


data_ = {}
data_['cameras']={}


argv = sys.argv

for n in tree.nodes:
    tree.nodes.remove(n)

rl = tree.nodes.new('CompositorNodeRLayers')
if engine=='CYCLES':
    d = tree.nodes.new('CompositorNodeDenoise')
    l1=links.new(rl.outputs['Image'], d.inputs[0])

v = tree.nodes.new('CompositorNodeComposite')
v.use_alpha = False


mm = tree.nodes.new('CompositorNodeMath')
mm.operation='MULTIPLY'
min_depth_=bpy.data.cameras[0].clip_start
mm.inputs[1].default_value=1.0/min_depth_

links.new(rl.outputs['Depth'], mm.inputs[0])

mp = tree.nodes.new('CompositorNodeMath')
mp.operation='POWER'
mp.inputs[1].default_value=-1
links.new(mm.outputs[0], mp.inputs[0])

render.image_settings.compression=0
#render.image_settings.exr_codec='ZIP'
render.image_settings.exr_codec='NONE'

resolution_x=render.resolution_x
resolution_y=render.resolution_y
lens=bpy.data.cameras[0].lens/1000
min_depth=bpy.data.cameras[0].clip_start
max_depth=bpy.data.cameras[0].clip_end
width=bpy.data.cameras[0].sensor_width/1000

data_['environment']={}
data_['environment']['name']=scene_name;
data_['environment']['width']=width
data_['environment']['resolution_x']=resolution_x
data_['environment']['resolution_y']=resolution_y
data_['environment']['lens']=lens
data_['environment']['max_depth']=max_depth
data_['environment']['min_depth']=min_depth
data_['environment']['lens']=lens

pi=math.pi
i=0
# iterate through objects
for obj_ in bpy.data.objects:
    if obj_.type=="CAMERA":


        name_=obj_.name
        name_rgb="rgb_"+name_
        name_depth="depth_"+name_

        path_rgb=scene_path_dtam+name_rgb
        path_depth=scene_path_dtam+name_depth

        camera=bpy.data.cameras[name_]
        scene.frame_current=i

        # scene.view_settings.exposure=2*math.sin((i/120)*8*pi)

        if engine=='CYCLES':
            l1=links.new(d.outputs['Image'], v.inputs[0])
        else:
            l1=links.new(rl.outputs['Image'], v.inputs[0])

        if( (con and not os.path.isfile(path_rgb+".png")) or not con ):
            render.image_settings.file_format='PNG'
            #render.image_settings.color_mode='RGB'
            render.image_settings.color_mode='BW'
            render.image_settings.color_depth='16'
            scene.view_settings.view_transform = 'Standard'
            render.engine=engine
            render.filepath = os.path.join(scene_path_dtam,name_rgb )
            bpy.ops.render.render(write_still = True)

        links.remove(l1)
        l2=links.new(mp.outputs[0], v.inputs[0])
        if( (con and not os.path.isfile(path_depth+".exr")) or not con ):
            render.image_settings.file_format='OPEN_EXR'
            render.image_settings.color_mode='BW'
            render.image_settings.color_depth='32'

            scene.view_settings.view_transform = 'Raw'
            render.engine=engine_eevee
            render.filepath = os.path.join(scene_path_dtam,name_depth )
            bpy.ops.render.render(write_still = True)


        pi=math.pi

        R_=obj_.rotation_euler.to_matrix()
        l_=obj_.location

        pose_=[ R_[0][0], R_[0][1], R_[0][2],
                R_[1][0], R_[1][1], R_[1][2],
                R_[2][0], R_[2][1], R_[2][2],
                l_[0], l_[1], l_[2] ]


        data_['cameras'][name_]={}
        data_['cameras'][name_]['frame']=i
        data_['cameras'][name_]['pose']=pose_


        with open(json_path, 'w') as outfile_:
            json.dump(data_, outfile_)

        i=i+1
