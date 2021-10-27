#Name of the .blend file inside "blender_scenes" directory
dataset_name='bunny_scene'
# dataset_name='classroom'

#Number of samples for CYCLES pathtracer
samples=128

cd ./blender_scenes

#Render with CYCLES engine
blender --background ./${dataset_name}.blend --python ./python_scripts/script.py CYCLES ${samples}

#Render with EEVEE engine
# blender --background ./${dataset_name}.blend --python ./python_scripts/script.py
