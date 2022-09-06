## Configuration

- [Launch Arguments](#launch-arguments)
- [YAML Parameters](#yaml-parameters)
- [Other](#other)
  - [MQTT Encryption](#mqtt-encryption)
  - [Docker Network Encryption](#docker-network-encryption)


### Launch Arguments

This section gives an overview of the possible variations that can be configured via ROS launch arguments. In order to reproduce the experiments from our paper, take a look at [`EXPERIMENTS.md`](EXPERIMENTS.md).

The arguments listed below can be added to the launch commands of the specific methods, e.g., `gui:=true`.

#### Any Method

| Argument | Description | Default | Type or Options |
| --- | --- | --- | --- |
| `type` | switch to launch components for vehicle or cloud role | - | `vehicle` / `cloud` |
| `gui` | whether to launch plotting and experiments GUI | `false` | bool |
| `record` | whether to record to a rosbag | `false` | bool |
| `packets` | topic of initial `velodyne_msgs::VelodyneScan` messages | `/vlp_32c/velodyne_packets` | string |
| `pointcloud` | topic of initial `sensor_msgs::PointCloud2` messages | - | string |
| `datatype` | toggle between sending `velodyne_msgs::VelodyneScan` or `sensor_msgs::PointCloud2` | `packets` | `packets` / `pointcloud` |
| `p_packets` | percentage of point cloud packets to send to allow downsampling | `1.0` | float |

#### `mqtt_client` / `mqtt_bridge`

| Argument | Description | Default | Type or Options |
| --- | --- | --- | --- |
| `host` | MQTT broker host | - | string |
| `port` | MQTT broker port | `1883` | int |
| `user` | MQTT broker username | - | string |
| `pass` | MQTT broker password | - | string |
| `qos` | MQTT QoS level | `0` | `0` / `1` / `2` |
| `ssl` | whether to enable SSL over MQTT | `false` | bool |
| `transmission_only` | whether to only measure transmission to broker and back | `false` | bool |


### YAML Parameters

Advanced configuration is not exposed via ROS launch arguments, but stored in parameter YAML files in [`src/benchmarking_suite/params`](src/benchmarking_suite/params/). A selection of these parameters is presented below.

| File | Parameter | Description | Default | Type |
| --- | --- | --- | --- | --- |
| [`benchmark_io.yaml`](src/benchmarking_suite/params/benchmark_io.yaml) | `bag_rate` | point cloud publication rate from bagfile [Hz] | `10` | float |
| [`object_detection.yaml`](src/benchmarking_suite/params/object_detection.yaml) | `latency` | processing latency of dummy object detection | `0.0` | float |
| [`object_detection.yaml`](src/benchmarking_suite/params/object_detection.yaml) | `n_objects` | number of objects in dummy object list | `20` | int |


## Other

### MQTT Encryption

The MQTT protocol supports exchanging data via SSL/TLS-encrypted communication channels. The encryption has to be configured on both client and broker sides.

#### Broker

Launch the MQTT broker [*Mosquitto*](https://mosquitto.org/) on port 8883 with a dedicated config file that enables encryption.

> **Warning**  
> Note that the default broker configuration that we provide ([`mosquitto.conf`](src/benchmarking_suite/params/mosquitto_ssl/mosquitto.conf)) is not secure. The authentication details and encryption certificates are public. Anyone can connect to your broker while it is running. See [Cleanup](README.md#cleanup) for instructions on how to shut down the broker.

```bash
# ros-v2x-benchmarking-suite/
docker run --rm -d \
--publish 8883:8883 \
--volume $(pwd)/src/benchmarking_suite/params/mosquitto_ssl:/mosquitto/config \
--user $(id -u):$(id -g) \
--name mosquitto \
eclipse-mosquitto
```

#### Client

For the MQTT-based methods, add `ssl:=true` to the benchmarking launch commands.


### Docker Network Encryption

All traffic via a Docker swarm overlay network can be AES-encrypted ([doc](https://docs.docker.com/network/overlay/#encrypt-traffic-on-an-overlay-network)). Encryption can be enabled when creating the Docker network.

#### Cloud

For the [`overlay`](README.md#overlay) method, create an encrypted [Docker overlay network](https://docs.docker.com/network/overlay/) to connect multiple containers within a Docker swarm.

```bash
docker network create --driver overlay --attachable --opt encrypted overlay
```
