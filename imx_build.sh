#!/bin/sh
cd ~/Desktop/Linux_for_tegra/source/kernel/kernel-jammy-src/
make M=drivers/media/i2c modules -j$(nproc)
sudo cp drivers/media/i2c/imx219.ko   /lib/modules/$(uname -r)/kernel/drivers/media/i2c/
#sudo depmod -a
cd ~/WIP/
