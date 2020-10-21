# VINS-Stereo
A stereo camera based VIO method, which extends and builds on VINS-Mono https://github.com/HKUST-Aerial-Robotics/VINS-Mono

# Compile 
Needs to build VINS-Mono first, since it depends on camera_model and pose_graph modules in VINS-Mono. 

# Run

roslaunch fpv.launch (euroc_stereo.launch or tum_vio.launch)    
rosbag play [data_sequence].bag   

data_sequence can be downloaded at   
https://fpv.ifi.uzh.ch/?page_id=50 for FPV dataset    
https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets for EuRoc MAV dataset   
https://vision.in.tum.de/data/datasets/visual-inertial-dataset for TUM VI dataset   

A demo using FPV data sequences is shown below:      

<a href="https://youtu.be/puo5BCzSXnQ" target="_blank"><img src="https://i.ytimg.com/vi/puo5BCzSXnQ/maxresdefault.jpg"
alt="UTH-FPV Dataset" width="240" height="180" border="10" /></a>


More details can be found in the report https://fpv.ifi.uzh.ch/wp-content/uploads/sourcenova/uni-comp/2019-2020-uzh-fpv-benchmark/submissions/15/details.pdf

# TODO:
Need to reduce the feature tracking drift to obtain more accurate feature matches between the stereo image pairs, as suggested in "A Comparative Analysis of Tightly-coupled Monocular, Binocular, and Stereo VINS".    
One promising solution is to replace FAST+KLT frontend with LIBVISO, an accurate dense stereo tracking method "StereoScan: Dense 3D Reconstruction in Real-time". 
... 
