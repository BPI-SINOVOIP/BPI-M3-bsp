sudo gunzip -c SD/100MB/BPI_M3_720P.img.gz | dd of=$1 bs=1024 seek=8
#sudo gunzip -c SD/100MB/BPI_M3_LCD7.img.gz | dd of=$1 bs=1024 seek=8
sync
cd SD/
sudo bpi-update -d $1
