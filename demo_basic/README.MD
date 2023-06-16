## About this project
This project is used to demostrate file system with fault tolerant enabled can be open by Linux.

## How to run
1. Open the project in codespace.
1. Create a raw disk and format to FAT.
   ```
   sudo apt-get update
   sudo apt-get install dosfstools
   dd if=/dev/zero of=raw.bin bs=1K count=65536
   mkfs.vfat raw.bin
   ```
1. Open a new terminal and build the project. Make sure the path is at the root of this repository.
   ```
   cmake -GNinja -Bbuild
   cmake --build build
   ```
1. Run demo_basic.
   ```
   ./build/demo_basic/demo_basic
   ```
   Expected output: 
   ```
   FileX basic demo complete
   ```
1. Mount and check the content in the disk.
   ```
   sudo mount -t vfat ./raw.bin /mnt && ls /mnt && cat /mnt/TEST.TXT
   ```
   Expected output:
   ```
   TEST.TXT
    ABCDEFGHIJKLMNOPQRSTUVWXYZ
   ```
1. Umount the disk.
   ```
   sudo umount /mnt
   ```