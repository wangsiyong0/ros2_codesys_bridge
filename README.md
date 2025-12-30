
参考代码：https://github.com/damuxt/ros2_arduino_bridge
# 回到工作空间
cd ~/ros2_ws
# 编译
colcon build --packages-select ros2_codesys_bridge
# source环境
source install/setup.bash
# 启动节点
ros2 launch ros2_codesys_bridge ros2_codesys.launch.py

echo "# ros2_codesys_bridge" >> README.md
git init
git add README.md
git commit -m "first commit"
git branch -M main
git remote add origin https://github.com/wangsiyong0/ros2_codesys_bridge.git
git push -u origin main