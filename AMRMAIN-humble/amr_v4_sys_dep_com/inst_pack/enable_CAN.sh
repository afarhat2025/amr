#!/bin/bash
sudo busybox devmem 0x0c303000 32 0x0000C400
sudo busybox devmem 0x0c303008 32 0x0000C458
sudo busybox devmem 0x0c303010 32 0x0000C400
sudo busybox devmem 0x0c303018 32 0x0000C458
sudo modprobe mttcan
sudo modprobe can
sudo modprobe can_raw
sudo ip link set down can0
sudo ip link set down can1
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can1 type can bitrate 125000
sudo ip link set up can0
sudo ip link set up can1
sudo ip link set can0 type can restart-ms 200
sudo ip link set can1 type can restart-ms 200
sudo ifconfig can0 txqueuelen 1000
sudo ifconfig can1 txqueuelen 1000
sudo ifconfig eqos0 down
sudo ifconfig wlan0 down
sudo ifconfig eth0 down
sudo ifconfig eth0 192.168.0.80 netmask 255.255.255.0
sudo ifconfig eqos0 192.168.1.80 netmask 255.255.255.0
sudo ifconfig wlan0 10.10.253.151 netmask 255.255.255.0
#sudo ip route del default || true
#sudo ip route add default via 10.10.2.250 dev wlan0
#sudo resolvectl dns eno1 10.10.1.17
sudo ifconfig wlan0 up
sudo ifconfig eqos0 up
sudo ifconfig eth0 up
sudo jetson_clocks
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.core.wmem_max=16777216
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728 #zed recomm
sudo sysctl -w net.core.rmem_default=1048576
sudo sysctl -w net.core.wmem_default=1048576
sudo sysctl -w net.ipv4.ipfrag_time=3 # zed recom
sudo sysctl -w net.core.rmem_default=16777216 #rec
#sudo sysctl -w net.ipv4.tcp_rmem="4096 131072 33554432"
#sudo sysctl -w net.ipv4.tcp_wmem="4096 65536 16777216"
#sudo sysctl -w net.ipv4.udp_rmem_min=32768
#sudo sysctl -w net.ipv4.udp_wmem_min=32768
#sudo sysctl -w net.core.optmem_max=2097152
#sudo sysctl -w net.core.netdev_max_backlog=250000
#sudo sysctl -w net.ipv4.udp_mem="8388608 12582912 16777216"
#sudo sysctl -w net.core.wmem_default=4194304
#sudo sysctl -w net.core.wmem_max=33554432
##sudo sysctl -w net.core.rmem_default=33554432
#sudo sysctl -w net.core.rmem_max=2147483647
#sudo sysctl -w net.ipv4.udp_rmem_min=65536
#sudo sysctl -w net.ipv4.udp_wmem_min=65536
sudo sysctl -w net.ipv6.conf.all.disable_ipv6=1
sudo sysctl -w net.ipv6.conf.default.disable_ipv6=1
sudo sysctl -w net.ipv6.conf.lo.disable_ipv6=1
sudo iw wlan0 set power_save off
sudo /usr/sbin/nvpmodel -m 0
exit 0
