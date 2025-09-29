

厚型涂料喷涂设备测试

    # 安装libmodbus-dev通讯库（tcp与rtu通讯）
    sudo apt install libmodbus-dev

    # 安装系统的 hidapi 库
	sudo apt update
	sudo apt install libhidapi-hidraw0

    # 添加或者修改外设权限
	sudo vim /etc/udev/rules.d/99-hidraw-permissions.rules
	# 内容如下：
	KERNEL=="hidraw*", KERNELS=="0005:057E:2006.*", MODE="0666"
	KERNEL=="hidraw*", KERNELS=="0005:057E:2007.*", MODE="0666"
	# 加载规则及应用:
	sudo udevadm control --reload-rules
	sudo udevadm trigger
	# 检查权限：
	ls -l /dev/hidraw*

	# 安装终端控制库
	sudo apt install libncurses5-dev libncursesw5-dev

	# 测试底盘
	cd beien_paint
	source install/setup.bash
	# 1. 启动PLC通信节点
	ros2 launch pkg_plc_communicate_cpp plc.launch.py 
	# 2. 启动控制节点
	ros2 run pkg_beien_paint_cpp beien_paint 
	# 3. 外设控制
	# 3.1 采用 Joycon 手柄控制
	# 3.1.1 启动 Joycon Left 手柄（控制行走速度）
		ros2 run pkg_joystick_cpp joycon_left_pub
	# 3.1.2 启动 Joycon Right 手柄（控制转向角度）
		ros2 run pkg_joystick_cpp joycon_right_pub 
	# 3.2 采用 键盘 控制
		ros2 run pkg_keyboard_control_cpp keyboard_control