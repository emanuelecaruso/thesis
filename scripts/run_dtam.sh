#Name of the .blend file inside "blender_scenes" directory

dataset_name='bunny_scene'
# dataset_name='bunny_scene_exposure'
# dataset_name='bunny_scene_roll'
# dataset_name='scene_roll'
# dataset_name='bunny_scene_zoom'


cd prova_dr

# ./build/executables/test_tracking ${dataset_name}
./build/executables/test_mapping ${dataset_name}
# ./build/executables/eval_orb_initializer ${dataset_name}

# cuda-memcheck ./build/executables/test_mapping ${dataset_name}
# blender ../blender_scenes/${dataset_name}.blend --python ../blender_scenes/python_scripts/run.py
