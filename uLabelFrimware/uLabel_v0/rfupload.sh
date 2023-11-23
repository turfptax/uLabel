curdir="$PWD"
cd ~/robotics/uECG/nodejs_bootloader_usb/
node cmd_fw_uploader_mk2.js $curdir/build/uLabel.bin 
cd $curdir

