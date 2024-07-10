# Antonio ripo
simple kartographer robot for 3D mapping project

# dipendency
pico-sdk ripo --> https://github.com/raspberrypi/pico-sdk.git

micro-ros pico-sdk ripo --> https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git

freeRTOS-Kernel ripo --> https://github.com/FreeRTOS/FreeRTOS-Kernel.git

rplidar_ros2 ripo --> https://github.com/babakhani/rplidar_ros2.git

# Running commands
micro-ros-agent serial --dev /dev/<Device name Such as ; 'ttyACM0'> -b 115200

		----------------fix USB-------------------
# open file to fix USB  
	< sudo nano /etc/udev/rules.d/10-usb-serial.rules >
# and enter
	SUBSYSTEM=="tty", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", SYMLINK+="lidar_A1"
	SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000a", ATTRS{serial}=="E6614C311B494831", SYMLINK+="pi_pico_CCD"
	SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000a", ATTRS{serial}=="E661640843316822", SYMLINK+="pi_pico_defDriv"
	SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="9c5016f8e689ec11b075b971d76262f7", SYMLINK+="rplidar_S1"
	SUBSYSTEM=="tty", ATTRS{idVendor}=="2639", ATTRS{idProduct}=="0300", ATTRS{serial}=="DB92OR80", SYMLINK+="Left_imu_xsens"
	SUBSYSTEM=="tty", ATTRS{idVendor}=="2639", ATTRS{idProduct}=="0300", ATTRS{serial}=="DB92OQE4", SYMLINK+="Right_imu_xsens"


docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/pi_pico_CCD -b 115200
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/pi_pico_defDriv -b 115200
	

