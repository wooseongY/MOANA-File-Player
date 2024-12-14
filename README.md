# File player for MOANA dataset in ''ROS2''

This program is a file player for the [MOANA dataset](https://sites.google.com/view/rpmmoana). Please use the issue in this github page to report a bug or ask questions.


## 1. How to install, Build, and Run
```
$ mkdir -p catkin_ws/src
$ cd catkin_ws/src
$ git clone -b [ros2] https://github.com/wooseongY/MOANA-File-Player.git
$ cd .. && colcon build
$ source install/setup.bash
$ ros2 launch moana_file_player moana_file_player.launch
```
- This version is tested in ROS-humble (Ubuntu 22.04)


## 2. Prepare the data and timestamps
Before loading the data, you need to generate data_stamp.csv with timestamp_gen.py.

You have to change the base_dir in timestamp_gen.py to the appropriate path for your data.
```
$ python3 timestamp_gen.py
```
Then, data_stamp.csv file will be created in your data directory.

If your data directory is represented as follows, you are now ready to enjoy the MOANA dataset!
```
📂 Sequence_name/
├── 📂 sensor_data/
│   ├── 📂 Camera_left/
│   │   └── 📝 timestamp.jpg
│   ├── 📂 Camera_right/
│   │   └── 📝 timestamp.jpg
│   ├── 📂 LiDAR/
│   │   └── 📝 timestamp.bin
│   ├── 📂 W_band_radar/
│   │   └── 📝 timestamp.png
│   ├── 📂 X_band_radar/
│   │   └── 📝 timestamp.png
└── 📝 data_stamp.csv
```

## 3. Load data files and play

1. Click the "Load" button.
2. Choose Sequence_name folder including sensor_data folder and data_stamp.csv.
3. The "Play" button starts publishing data in the ROS message.
4. The "Pause/Resume" button pauses and resumes publishing data.
5. The "Save" button saves all topics into the rosbag file.
6. The "Loop" checkbox resumes when playback is finished.

Enjoy it:) 

## Maintainer

Wooseong Yang (yellowish@snu.ac.kr)
Hyesu Jang (dortz@snu.ac.kr)

