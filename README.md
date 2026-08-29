# Go2 + MID360 portable ROS 2 stack

Target: Ubuntu 22.04, ROS 2 Humble, CycloneDDS, Livox MID360, and FAST-LIO2.
The same source pins are intended for amd64 validation and arm64 Orin NX deployment.

## Build

```bash
./scripts/fetch_sources.sh
docker-compose build
docker-compose run --rm ros2 ./scripts/build.sh
docker-compose run --rm ros2 ./scripts/verify.sh
```

## Live device

Before starting, connect MID360 and bring up the `MID360-static` NetworkManager
profile on the USB Ethernet NIC. The validated host address is
`192.168.123.5/24` and the lidar address is `192.168.123.170`. Discovery data
identifies the connected unit as device type 35 (MID360S), so the active driver
configuration is `config/MID360S_config.json`.

Terminal 1:

```bash
docker-compose run --rm ros2 ./scripts/run_driver.sh
```

One-command live sensor validation:

```bash
docker-compose run --rm ros2 ./scripts/test_mid360_live.sh
```

Terminal 2:

```bash
docker-compose run --rm ros2 ./scripts/run_mapping.sh
```

Live driver + FAST-LIO2 + 10-second MCAP validation:

```bash
docker-compose run --rm ros2 ./scripts/test_mapping_live.sh
```

The project-owned FAST-LIO2 configuration disables automatic PCD accumulation
by default to avoid exhausting memory during long tests. Edit
`config/fastlio_mid360.yaml` for a bounded map-saving run.

Terminal 3 (recommended during every hardware test):

```bash
docker-compose run --rm ros2 ./scripts/record.sh
```

Do not use a PoE switch or PoE injector to power the MID360.

## C12 visible and thermal streams

Assign an address such as `192.168.144.10/24` to the NIC connected to C12,
then run this script on the Ubuntu host (outside Docker):

```bash
./scripts/test_c12.sh
```

It probes the visible RTSP stream on port 554 and thermal stream on port 555,
then saves one PNG from each stream under `~/桌面/C12_验证结果`.
