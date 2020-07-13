## **BPI-M3-bsp**
Banana Pi M3 board bsp (u-boot 2014.7 & Kernel 3.4.39)


----------
**Prepare**

Get the docker image from [Sinovoip Docker Hub](https://hub.docker.com/r/sinovoip/bpi-build/) , Build the source code with this docker environment.

 **Build**

Build a target board bsp packages, please run

`#./build.sh 1`

Target download packages in SD/ after build. Please check the build.sh and Makefile for detail

**Install**

Get the image from [bpi](http://wiki.banana-pi.org/Banana_Pi_BPI-M3#Image_Release) and download it to the SD card. After finish, insert the SD card to PC and run this script in a normal terminal(not the docker image)

    # ./update_bsp2sd.sh /dev/sdX
