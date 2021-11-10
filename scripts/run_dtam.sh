#Name of the .blend file inside "blender_scenes" directory

# dataset_name='bunny_scene'
dataset_name='bunny_scene_roll'


cd prova_dr

# ./build/executables/dataset_maker
# ./build/executables/test_tracking ${dataset_name}
./build/executables/test_mapping ${dataset_name}
# ./build/executables/test_mapping_myalg ${dataset_name}

# cuda-memcheck ./build/executables/test_mapping ${dataset_name}
# blender ../blender_scenes/${dataset_name}.blend --python ../blender_scenes/python_scripts/run.py
