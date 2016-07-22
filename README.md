# BPI-M3-bsp

Getting Started
---------------

1. Choose a board doing:
   `./build.sh`,

2. Choose a board doing

"This tool supports the following BPI board(s):"

************************************************"

	1. BPI_M3_720P"

	2. BPI_M3_1080P"
	
	3. BPI_M3_LCD7"
	
	4. BPI_M3_USB_720P"
	
	5. BPI_M3_USB_1080P"
	
*************************************************"


BPI-M3 SD Card Info
--------------------

Step 1.To be on safe side erase the first part of your SD Card (also clears the partition table).


       sudo  dd if=/dev/zero of=${card} bs=1M count=1

Step 2.Go to folder "Download", put the file(s) to 100MB of the SD Card with DD command.



	 sudo dd if=boot0_sdcard.fex     of=${card} bs=1k seek=8

	sudo dd if=u-boot.fex 	        of=${card} bs=1k seek=19096
	
	sudo dd if=sunxi_mbr.fex 	of=${card} bs=1k seek=20480
	
	sudo dd if=boot-resource.fex 	of=${card} bs=1k seek=36864

	sudo dd if=env.fex 		    of=${card} bs=1k seek=69632

	sudo dd if=boot.fex 		of=${card} bs=1k seek=86016




Overview
--------
You may go to http://www.banana-pi.org to download Ubuntu15.04 images and burn them into the Sdcard to see the architecture.





