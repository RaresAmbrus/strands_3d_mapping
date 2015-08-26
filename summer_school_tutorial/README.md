Local metric map / semantic map
=================================

# Description

This set of nodes allows the creation, calibration, storage and querying of local metric maps as well as the detected dynamic elements. Recorded data is stored in `~/.semanticMap/` and the number of observations stored for each waypoint can be specified as a launch file parameter, as well as whether the observations should be stored in mongodb. Detailed information about how the individual nodes function, input/output topics and a parameters can be found in the corresponding readme files.  

### Start all the components

* Dependencies:

```
roslaunch mongodb_store mongodb_store.launch
rosrun scitos_ptu ptu_action_server_metric_map.py
```

* System nodes:
```
roslaunch semantic_map_launcher semantic_map.launch
```


# Doing a metric sweep

The underlying data used by the nodes in this package is obtained by performing a sweep of the Pan-Tilt Unit and collecting RGBD data at a number of positions during the sweep. 

### Relevant nodes
* ```rosrun cloud_merge do_sweep.py```
* ```roslaunch cloud_merge cloud_merge.launch```
* ```rosrun scitos_ptu ptu_action_server_metric_map.py```

### Operation

To start a sweep use the `do_sweep` action server:
```
rosrun actionlib axclient.py /do_sweep
```

This action server takes as input a string, with the following values defined: `complete`, `medium`, `short`, `shortest`. Internally the action server from `scitos_ptu` called `ptu_action_server_metric_map.py` is used, so make sure that is running.

The behavior is the following:

* If sweep type is `complete`, the sweep is started with parameters `-160 20 160 -30 30 30` -> 51 positions
* If sweep type is `medium`, the sweep is started with parameters `-160 20 160 -30 30 -30` -> 17 positions
* If sweep type is `short`, the sweep is started with parameters `-160 40 160 -30 30 -30` -> 9 positions
* If sweep type is `shortest`, the sweep is started with parameters `-160 60 140 -30 30 -30` -> 6 positions (there might be blank areas with this sweep type, depending on the environment).


The cloud_merge_node acquires data from the RGBD sensor, as the PTU unit does a sweep, stopping at various positions. As the PTU stops at a position, a number of RGBD frames are collected and averaged, with the purpose of reducing noise. Each one of these frames are converted to the global frame of reference, and merged together to form an observation point cloud, which is further processed by the `semantic_map` `semantic_map_node`.

If the sweep intermediate positions have been calibrated (using the `calibrate_sweeps` `calibrate_sweep_as action server`) and the parameter `register_and_correct_sweep` is set to `true`, the collected sweeps are also registered. Note that this registration is for the intermediate point clouds making up the sweep, and not between two sweeps.

The observations are stored on the disk, in the folder

`~.semanticMap/`


# Intermediate cloud calibration

As described earlier, each observation consists of a number of point clouds acquired at intermediate positions during a sweep. However, the exact pose of the camera optical center (camera frame of reference) with respect to the body of the robot (the odometry frame of reference) is not known exactly apriori, and hence when merging together the intermediate clouds misalignment errors often occur. To account for this, a calibration procedure has been developed that considers data from all observations of a particular type recorded so far. 

### Relevant nodes

```
rosrun calibrate_sweeps calibrate_sweep_as
```

### Operation

To start the calibration procedure, call: 

```
rosrun actionlib axclient.py /calibrate_sweeps
```

The calibration process uses only observations recorded with the type `complete` if using the `do_sweeps.py` action server from the `cloud_merge` package, i.e. with 51 positions.

If using the `ptu_action_server_metric_map.py` action server from the `scitos_ptu` package to record observations, the parameters are `-160 20 160 -30 30 30`.

Sweeps recorded with different parameters are ignored for the calibration. For registration, sweeps with different parameters are also processed if their parameters are a subset of the `complete` sweep type parameters (e.g. `comlpete` sweep type parameters are `-160 20 160 -30 30 30`; an example subset of those would be `-160 40 160 -30 30 30`, i.e. fewer pan stops).

### Calibration results

The calibration results are saved in `~/.ros/semanticMap`. These are:

* `registration_transforms.txt` the result of the 51 transforms for the intermediate poses.
* `registration_transforms_raw.txt` legacy - contains the same data as above in a different format, needed by the `strands_sweep_registration` package.
* `camera_params.txt` contains the optimized camera parameters. This is currently disabled, and the stored camera parameters should be the same as the input camera parameters.
* `sweep_paramters.txt` the sweep parameters used by the calibration (`-160 20 160 -30 30 30`)


# Metarooms and dynamic clusters

A meta-room contains only those parts of the scene which are observed to be static, and it is created incrementally as the robot re-observes the same location over time. Over time, as the robot visits the same location multiple times, the static part is modelled, and it is used to extract what is dynamic from subsequent observations. 

This package takes room observations as they are constructed by the `cloud_merge` package and extracts the corresponding metaroom and also computes dynamic clusters at each waypoint. Note that one metaroom is constructed for each waypoint. 

Some data is stored on the disk, in the folder

```
~.semanticMap/metarooms
```

### Relevant nodes

```
roslaunch semantic_map semantic_map.launch
```

### Operation

Once a new sweep has been performed at a waypoint, the corresponding metaroom is loaded (if it exists, otherwise a metaroom is initialized with the new observation), and it is used to extract the dynamic clusters/objects at that waypoint. 

### Sweep registration

The NDT algorithm is used to register obseravtions taken at the same waypoint together. The initial guess is provided by the AMCL robot pose, however, depending on the scene, the amount of changes between observations and sensor noise, sometimes the observations are poorly registered, resulting in poor dynamic clusters. One alternative in that case is to reset the metarooms, using the service described in the next section.

### ClearMetaroomService

This service resets the metarooms at specific waypoints.

Message type:

```
string[] waypoint_id
bool initialize
---
```

If initialize is set to `True`, the Meta-Rooms at the specific waypoints are re-initialized with the latest observations collected at those waypoints. Otherwise, the Meta-Rooms are just deleted.

### All vs most recent clusters

The default behavior of this node is to return all the dynamic clusters in an observation, as compared to the corresponding metaroom. However, sometimes this may mean a lot of information, and for that the parameter `newest_dynamic_clusters` is provided (default value `False`) - if set to `True`, the node will compute dynamic clusters by comparing the latest sweep with the previous one (as opposed to comparing the latest sweep to the metaroom), thus reporting only the latest dynamic clusters.




# Requesting dynamic clusters (package `object_manager`)

This package allows interaction with the dynamic clusters segmented by the `semantic_map` package

### Relevant nodes

```
rosrun object_manager object_manager_node
```

### DynamicObjectsService

Message tpe:
```
string waypoint_id
---
string[] object_id
sensor_msgs/PointCloud2[] objects
geometry_msgs/Point[] centroids
```

Given a waypoint id, this service returns all the dynamic clusters segmented at that waypoint, with their ids, point clouds and centroid. 

Service topic: `ObjectManager/DynamicObjectsService`

The point cloud corresponding to all the dynamic clusters is also published on the topic `"/object_manager/objects` 

###  GetDynamicObjectService

Message type:
```
string waypoint_id
string object_id
---
sensor_msgs/PointCloud2 object_cloud
int32[] object_mask
geometry_msgs/Transform transform_to_map
int32 pan_angle
int32 tilt_angle
```

Given a waypoint id and a cluster id (should correspond to the ids received after calling the `DynamicObjectsService`), this service returns the point cloud corresponding to that dynamic cluster in the camera frame of reference and a transform to get the point cloud in the map frame of refence. In addition, a set of angles (`pan_angle`, and `tilt_angle`) to which to turn the PTU, and a set of indices representing image pixels corresponding to the dynamic cluster in the image obtained after turning the PTU to the specified angles. 
After calling this service, the requested dynamic cluster is "selected", and after receiving the `start_viewing` mesasge on the `object_learning/status` topic, additional views received on the `/object_learning/object_view` topic will be added and logged together with this cluster.

Service topic: `ObjectManager/GetDynamicObjectService`

The point cloud corresponding to the requested dynamic cluster is also published on the topic `/object_manager/requested_object`.

The cluster mask is also published as an image on the topic: `/object_manager/requested_object_mask`

Note that the clusters are logged to the database when calling the `DynamicObjectsService` or  the `GetDynamicObjectService` (if the `log_to_db` argument is set to `True`). Calling these services multiple times does not affect (negatively) the logging. 

# Accessing observation data online (package `semantic_map_publisher`)

This package provides an interface to observation data previously recorded and stored on the disk. The data can be queried using the services described below (note that only some of the services available are described below; for more information please refer to the package description).

### Relevant nodes

```
rosrun semantic_map_publisher semantic_map_publisher
```


### WaypointInfoService

Message type:

```
---
string[] waypoint_id
int32[] observation_count
```

Returns a list of waypoints along with the number of observations collected at those waypoints.

Service name: `SemanticMapPublisher/WaypointInfoService`

## ObservationService
 
Message type:

```
string waypoint_id
float64 resolution
---
sensor_msgs/PointCloud2 cloud
```

Given a waypoint, and a resolution, this service returns the latest observation collected at that waypoint as a PointCloud with the specified resolution. 

Service name: `SemanticMapPublisher/ObservationService`

## WaypointTimestampService

Message type:

```
string waypoint_id
---
string[] waypoint_timestamps
```

Given a waypoint, this service returns the timestamps of all the observations collected at that waypoint, as a list. 

Service name: `SemanticMapPublisher/WaypointTimestampService`

## ObservationInstanceService

Message type:
```
string waypoint_id
int64 instance_number # convention 0 - oldest available
float64 resolution
---
sensor_msgs/PointCloud2 cloud
string observation_timestamp
```

Given a waypoint id, an instance number and a resolution, this service returns a particular instance from the observations collected at that particular waypoint, with the desired resolution, along with the timestamp of the observation (as opposed to `ObservationService` which returns the latest observation at that particular waypoint). 
Service name: `SemanticMapPublisher/ObservationInstanceService`


# Accessing data stored on the disk (package `metaroom_xml_parser`)

The `metaroom_xml_parser` package is used to parse previously saved room observations. The data will be read into an appropriate data structure containing: merged point cloud, individual point clouds, individual RGB and depth images and corresponding camera parameters.

## Utilities

A number of utilities are provided by this package, for easy data manipulation. The definitions can be seen in the file `load_utilities.h`

### Merged cloud utilities

The complete cloud datatype is:

```template <class PointType> boost::shared_ptr<pcl::PointCloud<PointType>>```

The utilities for loading only the merged cloud are:
* `loadMergedCloudFromSingleSweep` # returns one cloud
* `loadMergedCloudFromMultipleSweeps` # returns a vector of merged clouds, one for each sweep
* `loadMergedCloudForTopologicalWaypoint` # same as above
 
### Intermediate cloud utilities

The intermediate cloud datatype is:
```
    template <class PointType>
    struct IntermediateCloudCompleteData
    {
        std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>  vIntermediateRoomClouds;
        std::vector<tf::StampedTransform>                           vIntermediateRoomCloudTransforms;
        std::vector<image_geometry::PinholeCameraModel>             vIntermediateRoomCloudCamParams;
        std::vector<cv::Mat>                                        vIntermediateRGBImages; // type CV_8UC3
        std::vector<cv::Mat>                                        vIntermediateDepthImages; // type CV_16UC1
    };
```

The utilities for loading the intermediate clouds are:
* `loadIntermediateCloudsFromSingleSweep`                  # just the point clouds
* `loadIntermediateCloudsCompleteDataFromSingleSweep`      # complete data, with transforms and images
* `loadIntermediateCloudsFromMultipleSweeps`
* `loadIntermediateCloudsCompleteDataFromMultipleSweeps`
* `loadIntermediateCloudsForTopologicalWaypoint`
* `loadIntermediateCloudsCompleteDataForTopologicalWaypoint`
 

### Sweep XML utilities

The sweep XML is an `std::string`

The utilities for finding sweep XMLS are:
* `getSweepXmls` # takes a folder where to search as argument. Returns a `vector<string>`
* `getSweepXmlsForTopologicalWaypoint`

### Dynamic cluster utilities

The dynamic clusters type is:

```template <class PointType> std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>```

The dynamic cluster  utilities are:
* `loadDynamicClustersFromSingleSweep`
* `loadDynamicClustersFromMultipleSweeps`
* `loadDynamicClustersForTopologicalWaypoint`
 

# Data collected so far in Strands and available online

A number of datasets (in the formats described above) have been collected during Strands and made publicly available. For more information look at:

```
https://strands.pdc.kth.se/public/
```
