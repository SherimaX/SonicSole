IMU Backup
==========

This directory holds the IMU-enabled versions of the SonicSole Pi-side files
that were active at the moment IMU was removed from the build.

To restore IMU support:

1. Copy the files in this directory back to the repo root, overwriting the
   current pressure-only versions:

       cp imu_backup/SonicSole.h .
       cp imu_backup/SonicSole_IMU.cpp .
       cp imu_backup/SonicSole_Core.cpp .
       cp imu_backup/SonicSole_RPi.cpp .
       cp imu_backup/Makefile .
       cp imu_backup/RPi_combined_Header.h .
       cp imu_backup/RPi_Raj_Header.h .

2. Rebuild:

       make clean && make

3. The server (web_page/app.py) already handles 36-byte (with quaternion) and
   20-byte (acceleration only) packets. The pressure-only-aware path added at
   the same time as this backup is backwards compatible.

Files included
--------------
- SonicSole.h               IMU fields, mutex, thread, IMUSnapshot struct
- SonicSole_IMU.cpp         readIMU(), background-thread management, snapshot
- SonicSole_Core.cpp        Constructor/destructor IMU port handling
- SonicSole_RPi.cpp         Main loop with IMU snapshot + 9-float send
- Makefile                  Build target including SonicSole_IMU.cpp
- RPi_combined_Header.h     YEI command/streaming constants
- RPi_Raj_Header.h          YEI low-level helpers + IMU global
- serial_test.cpp           Standalone IMU sanity-check utility
