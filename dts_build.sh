#!/bin/sh
dtc -O dtb -o imx219_fixed.dtbo -@ ur_thermal.dts
#sudo fdtoverlay -i /boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv-super_COPY.dtb -o /tmp/merged_fix.dtb imx219_fixed.dtbo
sudo fdtoverlay -i kernel_tegra234-p3768-0000+p3767-0005-nv-super_COPY.dtb -o /tmp/merged_fix.dtb imx219_fixed.dtbo
sudo cp /tmp/merged_fix.dtb /boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv-super.dtb
