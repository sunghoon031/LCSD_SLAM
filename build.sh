echo "###################"
echo "[1] Build ORB-SLAM2"
echo "###################"
cd ~/LCSD_SLAM/ORB_SLAM2

echo "Configuring and building Thirdparty/DBoW2 ..."
cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

echo "Configuring and building Thirdparty/g2o ..."
cd ../../g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

echo "Uncompress vocabulary ..."
cd ../../../
cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM2 ..."
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

echo "#######################"
echo "[2] Build ORB-SLAM2 ROS"
echo "#######################"

echo "Building ROS nodes"

# Add ROS_PACKAGE_PATH to .bashrc unless you've already done it.
grep -q -F 'export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/LCSD_SLAM/Examples/ROS' ~/.bashrc || echo 'export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/LCSD_SLAM/Examples/ROS' >> ~/.bashrc

cd Examples/ROS/ORB_SLAM2
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j$(nproc)

echo "##############"
echo "[3] Build DSO "
echo "##############"

cd ~/LCSD_SLAM/DSO
mkdir build
cd build
cmake ..
make -j$(nproc)

echo "##################"
echo "[3] Build DSO ROS "
echo "##################"
catkin_make -DCMAKE_BUILD_TYPE=Release
