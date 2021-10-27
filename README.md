# DENSE TRACKING AND MAPPING

Application of the paper

Dense Tracking and Mapping in Real-Time

Richard A. Newcombe, Steven J. Lovegrove and Andrew J. Davison
Department of Computing, Imperial College London, UK


## INSTALL  (Ubuntu 18.04 LTS)


The following packages are required for the installation

###### CMake 3.15:

    install CMake 3.15

        https://cmake.org/files/v3.15/

###### Eigen3 stable version 3.3.9:

    download stable version 3.3.9 and install it with cmake

        http://eigen.tuxfamily.org

###### CUDA toolkit version 9.1

    sudo apt install nvidia-cuda-toolkit=9.1.85-3ubuntu1

###### OpenCV version 4.2 (with CUDA):

    be sure that CUDA toolkit is already installed (previous step)

    to install Opencv 4.2 with CUDA toolkit 9.2, use gcc and g++ version 6.5

  	Disable CUDNN by removing these lines in the guide:

        -D CUDA_ARCH_BIN=7.5 \
    	-D WITH_CUDNN=ON \
        -D OPENCV_DNN_CUDA=ON \

  	be sure that the path to opencv_contrib-4.5.2/modules is correct in the guide

    installation guide:

        https://gist.github.com/raulqf/f42c718a658cddc16f9df07ecc627be7

###### (Optional) Blender 3.90

    for generating new datasets, download and install Blender 3.90

        https://download.blender.org/release/


## COMPILATION

just run build_dtam.sh script inside directory: "scripts"

to change number of threads involved during the compilation, edit build_dtam.sh
by default 8 threads are used: make -j8


## EXECUTION


just run run_dtam.sh script inside directory: "scripts"

to run different datasets, edit run_dtam.sh by changing "dataset_name" variable


## GENERATE DATASET


To generate new Datasets, just a blender scene saved as .blend file is needed inside "blender_scenes" directory

To generate a set of cameras, the "create_cameras_in_scene.py" python script should be runned inside blender (scripting section)

Notice that for different set of cameras, "create_cameras_in_scene.py" has to be edited

After the file .blend is saved, to generate the dataset for dtam, run "generate_dataset.sh" inside directory: "scripts"

inside "generate_dataset.sh" we can choose:

    the dataset by assigning to "dataset_name" variable the name of the .blend file

    the engine (CYCLES = pathtracer) and the number of samples for CYCLES
# thesis
