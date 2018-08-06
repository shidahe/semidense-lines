# Incremental 3D Line Segment Extraction for Surface Reconstruction from Semi-dense SLAM 

This repo presents the code for my [Master's thesis](http://webdocs.cs.ualberta.ca/~vis/thesis_shida/). Some of the work is also presented in the following paper (accepted at ICPR 2018):

* **Incremental 3D Line Segment Extraction from Semi-dense SLAM**, Shida He, Xuebin Qin, Zichen Zhang, Martin Jagersand  ([arxiv](https://arxiv.org/abs/1708.03275))

Our method simplifies the per-keyframe pointcloud produced by a semi-dense SLAM system using 3D line segments. Then, we take the extracted 3D line segments and reconstruct a surface using them. In this way, surface of the scene viewed by the camera can be reconstructed while the camera is exploring. Our method produce accurate 3D line segments with few outliers, which makes the reconstructed surface more structually meaningful than surface reconstructed using feature points or random selected points. See my [thesis page](http://webdocs.cs.ualberta.ca/~vis/thesis_shida/) for more details.

This version of software is based on [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2). We use the semi-dense version of ORB-SLAM which is implemented based on the technique described in [this paper](https://www.researchgate.net/profile/Raul_Mur-Artal/publication/282807894_Probabilistic_Semi-Dense_Mapping_from_Highly_Accurate_Feature-Based_Monocular_SLAM/links/561cd04308ae6d17308ce267.pdf). We release this software under GPLv3 license. See [Dependencies.md](https://github.com/shidahe/semidense-lines/blob/master/Dependencies.md) for other dependencies.

If you use this software in an academic work, please cite:

    @inproceedings{he18icpr,
      title={Incremental 3D Line Segment Extraction from Semi-dense SLAM},
      author={Shida He, Xuebin Qin, Zichen Zhang, Martin Jagersand},
      booktitle={International Conference on Pattern Recognition, {ICPR} 2018},
      year={2018}
    }

# 1. Prerequisites
The software is tested in **64-bit Ubuntu 14.04**. 

#### C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

#### Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

#### OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 3.1.0**.

#### Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

#### BLAS and LAPACK
[BLAS](http://www.netlib.org/blas) and [LAPACK](http://www.netlib.org/lapack) libraries are requiered by g2o (see below). On ubuntu:
```
sudo apt-get install libblas-dev liblapack-dev liblapacke-dev
```
#### RapidJSON 
[RapidJSON](http://rapidjson.org) needs to be installed from for instance this [github repository](https://github.com/Tencent/rapidjson):
```
git clone https://github.com/Tencent/rapidjson
cd rapidjson
mkdir build
cmake ..
make && sudo make install
```
#### Ceres-solver
[Ceres-solver](http://ceres-solver.org) needs to be installed from for instance this [github repository](https://ceres-solver.googlesource.com/ceres-solver): 
```
 git clone https://ceres-solver.googlesource.com/ceres-solver
 cd ceres-solver
 mkdir build
 cd build
 cmake ..
 make &&  sudo make install
```
**Warning : ** cmake version 3.5 is needed to compile ceres-solver.

#### glog
[glog](https://github.com/google/glog) needs to be installed from for instance this [github repository] ( ): 
```
 git clone https://github.com/google/glog.git
 cd glog
 mkdir build
 cd build
 cmake -DGFLAGS_NAMESPACE=google -DCMAKE_CXX_FLAGS=-fPIC ..
 make && sudo make install
```

#### DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

#### ROS (optional)
We provide some examples to process the live input of a monocular, stereo or RGB-D camera using [ROS](ros.org). Building these examples is optional. In case you want to use ROS, a version Hydro or newer is needed.

## Additional
Apart from the dependencies list above for ORB-SLAM2, additional libraries are needed.

#### CGAL: 
```
sudo apt-get install libcgal-dev 
```
#### Boost:
```
sudo apt-get install libboost-all-dev
```
#### EdgeDrawing (Included in Thirdparty folder):
We use [EdgeDrawing](http://ceng.anadolu.edu.tr/CV/EdgeDrawing/) edge detector in our system. The library is available as binary files. The version for 64-bit Linux is included in the *Thirdparty* folder.

#### EDLines and Line3D++ (Included in Thirdparty folder):
We compare our system against methods using [EDLines](http://ceng.anadolu.edu.tr/cv/EDLines/) and [Line3D++](https://github.com/manhofer/Line3Dpp). EDLines is avaiable as binary files and the version for 64-bit Linux is included in *Thirdparty* folder. We also include Line3D++ in the *Thirdparty* folder and it will be compiled when running the `build.sh` script. 



# 2. Building

Similar to ORB-SLAM2, `build.sh` can build the Thirdparty libraries and semi-dense ORB-SLAM2 with 3D line segment extraction and surface reconstruction. Please make sure you have installed all required dependencies (see section 1). 

Execute:
```
chmod +x build.sh
./build.sh
```
Sometimes the build will fail with errors related to c++11 standard, and simply running the script again should fix the issue. 

# 3.  Examples

The system can be run in the same way of running ORB-SLAM2. Here are examples for running on EuRoC and TUM-RGBD dataset (monocular). It can also run using ROS. See [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) for more details.

## EuRoC Dataset

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Execute the following first command for V1 and V2 sequences, or the second command for MH sequences. Change `PATH_TO_SEQUENCE` and `SEQUENCE` according to the sequence you want to run.
```
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.bin Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE/mav0/cam0/data Examples/Monocular/EuRoC_TimeStamps/SEQUENCE.txt 
```
```
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE/cam0/data Examples/Monocular/EuRoC_TimeStamps/SEQUENCE.txt 
```

## TUM Dataset

1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

2. Execute the following command. Change `TUMX.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder.
```
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.bin Examples/Monocular/TUMX.yaml PATH_TO_SEQUENCE_FOLDER
```


# 4. Results

In default configuration, the sparse SLAM system run once and then all other processing (including semi-dense mapping, 3D line segment extraction and surface reconstruction) happens offline afterwards. Note the offline processing can take a long time which makes the windows unresponsive. **Please wait until the processing is finished in order to save the results.** Online processing can be enabled by uncomment the *OnlineLoop* macro in *ProbabilityMapping.cc* file. However, this is not recommended due to the fact that real-time performance is not yet guaranteed.

The results are saved in the directory *results_line_segments* under a subdirectory named by the starting time. 

In each result directory:
- *info.txt* reports time usage and used parameters. 
- *model.obj* is the reconstructed mesh of the scene. 
- *semi_pointcloud.obj* is the raw semi-dense pointcloud from semi-dense mapping. 
- *line_segments.obj* contains the extracted 3D line segments before clustering. 
- *line_segments_clustered_incr.obj* contains the clustered line segments. 
- *line_segments_edlines.obj* contains the line segments extracted using decoupled line segment fitting. 
- Files with *Line3D++* in their names are created by Line3D++.

## Reproduction

To reproduce the results, please run the following commands. Please change `PATH_TO_SEQUENCE_*` to the corresponding dicrectory of the sequence. The figures are screenshots of models rendered in [MeshLab](http://www.meshlab.net/).

1. Fig 4: This figure shows results from 4 different sequences: *EuRoC  MAV  Vicon  Room  101*, *EuRoC  MAV  Machine  Hall  01*, *TUM RGBD fr3-large-cabinet*, *TUM RGBD fr1-room*. 
```
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.bin Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE_V101/mav0/cam0/data Examples/Monocular/EuRoC_TimeStamps/V101.txt 
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE_MH01/cam0/data Examples/Monocular/EuRoC_TimeStamps/MH01.txt 
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.bin Examples/Monocular/TUM3.yaml PATH_TO_SEQUENCE_fr3_large_cabinet
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.bin Examples/Monocular/TUM1.yaml PATH_TO_SEQUENCE_fr1_room
```

2. Fig 5: This figure contains result from sequence *TUM RGB-D fr3-structure-texture-near*. 
```
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.bin Examples/Monocular/TUM3.yaml PATH_TO_SEQUENCE_fr3_structure_texture_near
```

3. Fig 6: This figure shows reconstructed surfaces from sequence *EuRoC  MAV  Vicon  Room  101*. In default configuration, the surface model with line segments endpoints will be reconstructed. To reconstruct the surface model with map points, please change all `mpModeler->AddLineSegmentKeyFrameEntry(kf)` calls in *ProbabilityMapping.cc* to `mpModeler->AddKeyFrameEntry(kf)` and build again. Running the command again will reconstruct the surface using only ORB-SLAM map points. 
```
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.bin Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE_V101/mav0/cam0/data Examples/Monocular/EuRoC_TimeStamps/V101.txt 
```

4. Table 1: This table shows results from sequences: *EuRoC  MAV  Vicon  Room  101* and *EuRoC  MAV  Vicon  Room  201*. In order to calculate the distance, please use the MATLAB scripts in the *eval* folder.
```
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.bin Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE_V101/mav0/cam0/data Examples/Monocular/EuRoC_TimeStamps/V101.txt 
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.bin Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE_V201/mav0/cam0/data Examples/Monocular/EuRoC_TimeStamps/V201.txt 
```

5. Table 2: The table presents the number of vertices of results from sequences: *EuRoC  MAV  Vicon  Room  101*, *EuRoC  MAV  Machine  Hall  01*, *TUM RGBD fr3-large-cabinet*, *TUM RGBD fr1-room*. The number of vertices in *.obj* files can be checked using [MeshLab](http://www.meshlab.net/).
```
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.bin Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE_V101/mav0/cam0/data Examples/Monocular/EuRoC_TimeStamps/V101.txt 
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE_MH01/cam0/data Examples/Monocular/EuRoC_TimeStamps/MH01.txt 
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.bin Examples/Monocular/TUM3.yaml PATH_TO_SEQUENCE_fr3_large_cabinet
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.bin Examples/Monocular/TUM1.yaml PATH_TO_SEQUENCE_fr1_room
```

