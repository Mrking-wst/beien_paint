

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