### Paper Experiments

Below you find a table listing all experiments that are reported in our paper and how to launch the specific configurations.

Please make yourself familiar with the [Quick Start](README.md#quick-start) first. In order to keep the launch commands simple, create more aliases containing all fixed parameters.

<details style="margin-left: 20px">
<summary><i>Cloud</i></summary>

```bash
alias benchmark_cloud="
  docker run --rm -it \
  --name benchmarking-cloud \
  ros-v2x-benchmarking-suite"

alias benchmark_cloud_ol="
  docker run --rm -it \
  --name benchmarking-cloud \
  --network overlay \
  ros-v2x-benchmarking-suite"

alias benchmark_cloud_mqtt_client="
  benchmark_cloud roslaunch benchmarking_suite benchmark_mqtt_client.launch \
  type:=cloud host:=$BROKER_HOST user:=$BROKER_USER pass:=$BROKER_PASS"

alias benchmark_cloud_mqtt_bridge="
  benchmark_cloud roslaunch benchmarking_suite benchmark_mqtt_bridge.launch \
  type:=cloud host:=$BROKER_HOST user:=$BROKER_USER pass:=$BROKER_PASS"

alias benchmark_cloud_overlay="
  benchmark_cloud_ol roslaunch benchmarking_suite benchmark_overlay.launch \
  type:=cloud"
```

</details>

<details style="margin-left: 20px">
<summary><i>Vehicle</i></summary>

```bash
alias benchmark_vehicle="
  docker run --rm -it \
  --env DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume $(pwd)/rosbags:/root/ws/rosbags \
  --volume $(pwd)/src/benchmarking_suite/experiments:/root/ws/src/benchmarking_suite/experiments \
  --name benchmarking-vehicle \
  ros-v2x-benchmarking-suite"

alias benchmark_vehicle_ol="
  docker run --rm -it \
  --env DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume $(pwd)/rosbags:/root/ws/rosbags \
  --volume $(pwd)/src/benchmarking_suite/experiments:/root/ws/src/benchmarking_suite/experiments \
  --name benchmarking-vehicle \
  --network overlay \
  --env ROS_MASTER_URI="http://benchmarking-cloud:11311" \
  ros-v2x-benchmarking-suite"

alias benchmark_vehicle_mqtt_client="
  benchmark_vehicle roslaunch benchmarking_suite benchmark_mqtt_client.launch \
  type:=vehicle host:=$BROKER_HOST user:=$BROKER_USER pass:=$BROKER_PASS"

alias benchmark_vehicle_mqtt_bridge="
  benchmark_vehicle roslaunch benchmarking_suite benchmark_mqtt_bridge.launch \
  type:=vehicle host:=$BROKER_HOST user:=$BROKER_USER pass:=$BROKER_PASS"

alias benchmark_vehicle_overlay="
  benchmark_vehicle_ol roslaunch benchmarking_suite benchmark_overlay.launch \
  type:=vehicle"

alias benchmark_in_vehicle="
  benchmark_vehicle roslaunch benchmarking_suite benchmark_in_vehicle.launch"
```

</details>

| Section | Method | Table | Figure | Cloud Launch Command | Vehicle Launch Command | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| V.A | `mqtt_client` | I | 4 | `benchmark_cloud_mqtt_client` | `benchmark_vehicle_mqtt_client` |  |
| V.A | `mqtt_bridge` | I | 4 | `benchmark_cloud_mqtt_bridge` | `benchmark_vehicle_mqtt_bridge` |  |
| V.A | `overlay` | I | 4 | `benchmark_cloud_overlay` | `benchmark_vehicle_overlay` |  |
| V.A | `in_vehicle` | I | 4 | - | `benchmark_vehicle_in_vehicle` |  |
| V.A (Encryption) | `mqtt_client` | II | - | `benchmark_cloud_mqtt_client` | `benchmark_vehicle_mqtt_client` |  |
| V.A (Encryption) | `mqtt_client` | II | - | `benchmark_cloud_mqtt_client port:=8883 ssl:=true` | `benchmark_vehicle_mqtt_client port:=8883 ssl:=true` | [MQTT Encryption](CONFIGURATION.md#mqtt-encryption) |
| V.A (Encryption) | `mqtt_client` | II | - | `benchmark_cloud_mqtt_client` | `benchmark_vehicle_mqtt_client` | VPN enabled |
| V.A (Encryption) | `overlay` | II | - | `benchmark_cloud_overlay` | `benchmark_vehicle_overlay` |  |
| V.A (Encryption) | `overlay` | II | - | `benchmark_cloud_overlay` | `benchmark_vehicle_overlay` | [Docker Network Encryption](CONFIGURATION.md#docker-network-encryption) |
| V.A (QoS) | `mqtt_client` | III | - | `benchmark_cloud_mqtt_client` | `benchmark_vehicle_mqtt_client` |  |
| V.A (QoS) | `mqtt_client` | III | - | `benchmark_cloud_mqtt_client qos:=1` | `benchmark_vehicle_mqtt_client qos:=1` |  |
| V.A (QoS) | `mqtt_client` | III | - | `benchmark_cloud_mqtt_client qos:=2` | `benchmark_vehicle_mqtt_client qos:=2` |  |
| V.A (Data Size and Type) | `mqtt_client` | IV | - | `benchmark_cloud_mqtt_client qos:=1` | `benchmark_vehicle_mqtt_client p_packets:=0.0 qos:=1` |  |
| V.A (Data Size and Type) | `mqtt_client` | IV | - | `benchmark_cloud_mqtt_client` | `benchmark_vehicle_mqtt_client p_packets:=0.25` |  |
| V.A (Data Size and Type) | `mqtt_client` | IV | - | `benchmark_cloud_mqtt_client` | `benchmark_vehicle_mqtt_client p_packets:=0.5` |  |
| V.A (Data Size and Type) | `mqtt_client` | IV | - | `benchmark_cloud_mqtt_client` | `benchmark_vehicle_mqtt_client p_packets:=0.75` |  |
| V.A (Data Size and Type) | `mqtt_client` | IV | - | `benchmark_cloud_mqtt_client` | `benchmark_vehicle_mqtt_client` |  |
| V.A (Data Size and Type) | `mqtt_client` | IV | - | `benchmark_cloud_mqtt_client datatype:=pointcloud` | `benchmark_vehicle_mqtt_client datatype:=pointcloud` |  |
| V.B | `mqtt_client` | V | 5 | - | `benchmark_cloud_mqtt_client transmission_only:=true` |  |
| V.B | `mqtt_client` | V | 5 | - | `benchmark_cloud_mqtt_client transmission_only:=true p_packets:=0.0` |  |
