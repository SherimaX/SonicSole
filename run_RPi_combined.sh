#! /bin/bash
# 
# echo "Mounting SD card..." ;
# umount /dev/mmcblk1p1 ;
# mount /dev/mmcblk1p1 /media/rootfs/sd/log/ ;
# 
# printf "\033c"
# printf " "
# echo "Setting CPU performance level..." ;
#cpufreq-set -g performance ;
#cpufreq-set -g conservative ;
# cpufreq-set -g userspace ;
# cpufreq-set -f 1.54Ghz ;
# 
# printf "\033c"
#printf " "
#echo "Activating GPIO port 204..." ;
#echo 204 > /sys/class/gpio/unexport ;
#sleep 1 ;
#echo 204 > /sys/class/gpio/export ;
#echo out > /sys/class/gpio/gpio204/direction ;
# 
# printf "\033c"


echo " "
echo "Running test script for data logging..."
# cd /root/MDA/DataLogging

#echo "Please enter Session FileName: "
#read input_var


#
# print
# 
./C_RPi_combined #$input_var
