#!/bin/bash
#if not done type sudo chmod +x sys_dep_install.sh
#then run ./sys_dep_install.sh
# Update and install locale settings

set -e

cd ~/AMRMAIN/amr_v4_sys_dep_com/inst_pack

#start downloading files now due to race conditions
wget https://update.code.visualstudio.com/1.91.0/linux-arm64/stable
wget https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/4.2/ZED_SDK_Tegra_L4T36.4_v4.2.5.zstd.run
wget https://nvidia.box.com/shared/static/zvultzsmd4iuheykxy17s4l2n91ylpl8.whl -O torch-2.3.0-cp310-cp310-linux_aarch64.whl
wget https://nvidia.box.com/shared/static/u0ziu01c0kyji4zz3gxam79181nebylf.whl -O torchvision-0.18.0a0+6043bc2-cp310-cp310-linux_aarch64.whl

sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Install software-properties-common and add universe repository
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

# Install curl and ROS 2 repository setup
sudo apt update
sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt install -y ros-humble-ros-base
sudo apt install python3-colcon-common-extensions

# Install and configure OpenCV
sudo apt install -y zstd
chmod +x ZED_SDK_Tegra_L4T36.4_v4.2.5.zstd.run
./ZED_SDK_Tegra_L4T36.4_v4.2.5.zstd.run -- silent
sudo chmod 755 ./OpenCV-4-11-0.sh
./OpenCV-4-11-0.sh

# Install Microsoft SQL Server and tools
sudo curl https://packages.microsoft.com/keys/microsoft.asc | sudo tee /etc/apt/trusted.gpg.d/microsoft.asc
sudo curl https://packages.microsoft.com/config/ubuntu/$(lsb_release -rs)/prod.list | sudo tee /etc/apt/sources.list.d/mssql-release.list
sudo apt-get update
sudo ACCEPT_EULA=Y apt-get install -y msodbcsql18
sudo ACCEPT_EULA=Y apt-get install -y mssql-tools18
sudo apt-get install -y unixodbc-dev

# Install CTR Electronics repository and Phoenix6
sudo curl -s --compressed -o /usr/share/keyrings/ctr-pubkey.gpg "https://deb.ctr-electronics.com/ctr-pubkey.gpg"
sudo curl -s --compressed -o /etc/apt/sources.list.d/ctr${YEAR}.list "https://deb.ctr-electronics.com/ctr2024.list"
sudo apt update

sudo dpkg -i stable
sudo apt-get install -y jstest-gtk busybox libpcl-dev libpcap-dev libboost-dev gdebi-core libyaml-cpp-dev linuxptp libqt5core5a qtcreator qtbase5-dev qt5-qmake cmake phoenix6

#gpio
sudo apt install -y libgpiod-dev

pip3 install 'Cython<3'
pip3 install numpy
pip3 install torch-2.3.0-cp310-cp310-linux_aarch64.whl
pip3 install torchvision-0.18.0a0+6043bc2-cp310-cp310-linux_aarch64.whl
pip3 install pyodbc
pip3 install cv-bridge
pip3 install python-vlc
pip3 install pymongo
pip3 install git-filter-repo
pip3 install phoenix6
pip3 install sqlalchemy
sudo pip3 install -U jetson-stats

#ptp and service files
sleep 1
sudo tee /etc/linuxptp/hesai.conf > /dev/null <<EOL
[global]
twoStepFlag		1
logAnnounceInterval	1
logSyncInterval		0
logMinDelayReqInterval	0
logMinPdelayReqInterval	0
announceReceiptTimeout	3
syncReceiptTimeout	0
network_transport	UDPv4
delay_mechanism		E2E
time_stamping		hardware
tx_timestamp_timeout	1
[eth0]
EOL

sudo tee /etc/systemd/system/ptp4l.service > /dev/null <<EOL
[Unit]
Description=PTP4L Service
After=network.target

[Service]
Type=simple
ExecStartPre=/bin/sleep 5
ExecStart=/usr/sbin/ptp4l -f /etc/linuxptp/hesai.conf
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOL

sudo tee /etc/systemd/system/phc2sys.service > /dev/null <<EOL
[Unit]
Description=phc2sys Service
After=ptp4l.service

[Service]
Type=simple
ExecStartPre=/bin/sleep 2
ExecStart=/usr/sbin/phc2sys -c eth0 -s CLOCK_REALTIME -O 0
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOL

sudo systemctl daemon-reload
sudo systemctl enable ptp4l.service
sudo systemctl enable phc2sys.service
sudo systemctl start ptp4l.service
sudo systemctl start phc2sys.service

#can 
sudo modprobe mttcan
chmod 755 enable_CAN.sh
printf '%s\n' '#!/bin/bash' '/home/amr/AMRMAIN/Miss/inst_pack/enable_CAN.sh &''exit 0' | sudo tee -a /etc/rc.local
sudo chmod +x /etc/rc.local

#mongo c driver
sudo apt-get -y install libmongoc-dev
git clone https://github.com/mongodb/mongo-cxx-driver.git -b r3.7.0
cd mongo-cxx-driver/build
cmake .. -DCMAKE_BUILD_TYPE=Release
sudo make install

#realtime
sudo apt install linux-lowlatency
sudo tee -a /etc/security/limits.conf > /dev/null <<EOF
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock 102400
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock 102400
EOF
sudo addgroup realtime
sudo usermod -a -G realtime $(whoami)

#kernel changes!!! do not change nothing here
sudo apt-get install -y dkms
sudo git clone https://github.com/paroj/xpad.git /usr/src/xpad-0.4
sudo dkms install -m xpad -v 0.4
UDEV_RULE="/etc/udev/rules.d/99-logitech-f710.rules"
if [ ! -f "$UDEV_RULE" ]; then
    cat <<EOL | sudo tee "$UDEV_RULE" > /dev/null
# Linux udev rule file for Logitech F710 gamepads.

ACTION=="add", SUBSYSTEM=="input", KERNEL=="js?", ATTRS{name}=="Logitech Gamepad F710", \
    RUN+="/bin/jscal -s 8,1,0,0,0,16448,16448,1,0,0,0,16448,16448,1,0,0,0,16448,16448,1,0,0,0,16448,16448,1,0,0,0,16448,16448,1,0,0,0,16448,16448,1,0,0,0,16448,16448,1,0,0,0,16448,16448 /dev/$name", \
    RUN+="/bin/jscal -u 8,16,17,0,1,2,3,4,5,11,307,305,304,308,310,311,314,315,316,317,318 /dev/$name"

EOL
else
    echo "Udev rule already exists. Skipping creation."
fi
sudo udevadm control --reload-rules
#test, netwrok side
#echo "net.core.rmem_max=2147483647" | sudo tee -a /etc/sysctl.conf
#echo "net.ipv4.ipfrag_time=3" | sudo tee -a /etc/sysctl.conf
#echo "net.ipv4.ipfrag_high_thresh=134217728" | sudo tee -a /etc/sysctl.conf
#sudo sysctl -p

#ros2 packages
sudo apt-get install -y ros-humble-apriltag-msgs \
ros-humble-apriltag-ros \
ros-humble-behaviortree-cpp-v3 \
ros-humble-bondcpp \
ros-humble-imu-filter-madgwick \
ros-humble-pcl-ros \
ros-humble-rmw-cyclonedds-cpp \
ros-humble-robot-localization \
ros-humble-robot-state-publisher \
ros-humble-ros2-control \
ros-humble-ros2-controllers \
ros-humble-rviz2 \
ros-humble-sick-scan-xd \
ros-humble-transmission-interface \
ros-humble-twist-mux \
ros-humble-urdf \
ros-humble-xacro \
ros-humble-yaml-cpp-vendor \
ros-humble-rtabmap \
ros-humble-gtsam \
ros-humble-rqt* \
ros-humble-teleop-twist-keyboard \
ros-humble-twist-mux \
ros-humble-joy \
ros-humble-pointcloud-to-laserscan \
ros-humble-libpointmatcher \
ros-humble-nmea-msgs \
ros-humble-joint-state-publisher \
ros-humble-imu-tools \
ros-humble-test-msgs \
ros-humble-geographic-msgs \
ros-humble-point-cloud-transport \
ros-humble-teleop-twist-joy

#nav2 underlying libraries
sudo apt install -y libgraphicsmagick++-dev
sudo apt install -y xtensor-dev
sudo apt install -y ros-humble-behaviortree-cpp
sudo apt install -y libceres-dev
sudo apt install -y libompl-dev

#optional
sudo jetson_clocks
#add packages as needed, during colcon build

#do not install this package, as this will be local
sudo apt remove -y ros-humble-cv-bridge
sudo apt remove -y ros-humble-rtabmap*
sudo apt remove -y ros-humble-nav2*
sudo snap stop cups.cups-browsed
sudo snap stop cups.cupsd
sudo snap remove cups
rm stable torch-2.3.0-cp310-cp310-linux_aarch64.whl torchvision-0.18.0a0+6043bc2-cp310-cp310-linux_aarch64.whl ZED_SDK_Tegra_L4T36.4_v4.2.5.zstd.run 
rm -rf mongo-cxx-driver
echo "------------------------------------"
echo "script finished, going to reboot now"
echo "------------------------------------"
sleep 5

sudo reboot now
