# ROS Objects to CSV
Use this to convert Autoware DetectedObjects or TrackingObjects to CSV.

- **TrackedObjectsToCSV**: Subscribes to a topic for tracked objects and saves the data to a single CSV file.
- **DetectedObjectsToCSV**: Subscribes to a topic for detected objects and saves the data to multiple CSV files named by timestamp.

## How to use
### Run detection and tracking
Run something like
- Apollo + Autoware multi-object tracking

### Build with appropriate msgs
autoware auto:
```git clone git@github.com:MapIV/autoware_auto_msgs.git```

(optional) autoware_ai: 
```git clone git@gitlab.com:perceptionengine/autoware/autoware_iv_msgs.git branch ros2 --single-branch```

```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Run
Parameters:
* `csv_directory_path`: The directory where CSV files will be saved. Default is `/default/path/to/directory`.
* `input_topic`: The topic from which the node will subscribe to perception messages. Default is `/tracked_objects` for `TrackedObjectsToCSV` and `/detected_objects` for `DetectedObjectsToCSV`.

E.g.
```bash
ros2 launch objects_to_csv trackedobjectstocsv.launch.xml csv_directory_path:=/home/map4/Downloads
```

```bash
ros2 launch objects_to_csv trackedobjectstocsv.launch.xml csv_directory_path:=/home/map4/Downloads input_topic:/sensing/top/medium/rare/pandar_points
```



