steps:
cd leo_lidar_sim

# 2. Copy all updated files
cp ~/Downloads/FINAL_package.xml package.xml
cp ~/Downloads/CMakeLists.txt .
cp ~/Downloads/real_robot.launch.py launch/
cp ~/Downloads/slam_params_real.yaml config/
cp ~/Downloads/nav2_params_real.yaml config/
cp ~/Downloads/rviz_config.rviz config/
cp ~/Downloads/colour_params.csv config/
cp ~/Downloads/detection_node.py scripts/
cp ~/Downloads/start_nav2.sh scripts/
chmod +x scripts/*.sh scripts/*.py

# 3. Update .bashrc
cat ~/Downloads/bashrc_additions.txt >> ~/.bashrc
source ~/.bashrc

# 4. Build
cd ~/leo_ws
colcon build --packages-select leo_lidar_sim
source install/setup.bash



Not to forget:
# Use ruler to measure from base_link:
LiDAR: X=___, Y=___, Z=___
Camera: X=___, Y=___, Z=___

Get colour_params.csv from Teammate!

ls /dev/ttyUSB*
