# Velodyne VLP-16 Real-Time Decoder & Visualizer

Real-time 3D point cloud visualization for the Velodyne VLP-16 LiDAR sensor, built for the [Autonomous Vehicle Laboratory](https://autovehiclelab.com) at Cal Poly Pomona.

## Overview

A custom Python decoder and visualizer that parses raw UDP packets from the Velodyne VLP-16, reconstructs 3D point clouds in real time, and renders them in an interactive Open3D environment. Serves as the first stage of AVL's LiDAR perception pipeline on the AVL-002 autonomous go-kart platform.

## Performance

- **300,000+ points/sec** real-time processing
- **15 FPS** stable 3D visualization
- **Validated** against VeloView ground truth
- Runs on both x86 desktop and Jetson AGX Orin

## Features

- Real-time UDP packet decoding via the `velodyne_decoder` C++ backend (10x faster than pure Python)
- Height-based point cloud coloring
- Configurable range and angle filtering
- Live streaming, PCAP replay, and ROS bag playback
- Threaded capture/render architecture with proper synchronization
- Adjustable scan buffer for smooth or sharp display
- Simulation mode for testing without hardware

## Requirements

- Python 3.8+
- [velodyne_decoder](https://github.com/valgur/velodyne_decoder)
- Open3D
- NumPy

```bash
pip install velodyne-decoder open3d numpy
```

## Network Setup

1. Set your machine's IP to the same subnet as the VLP-16:
   ```bash
   sudo ip addr add 192.168.1.100/24 dev eth0
   ```
2. Verify connectivity (default sensor IP: `192.168.1.201`, AVL config: `192.168.13.11`):
   ```bash
   ping 192.168.1.201
   ```
3. Open the data port if needed:
   ```bash
   sudo ufw allow 2368/udp
   ```

## Usage

```bash
# Live from VLP-16 (default UDP port 2368)
python3 velodyne_decoder_final.py

# Test without hardware
python3 velodyne_decoder_final.py --simulate

# Replay a PCAP recording
python3 velodyne_decoder_final.py --pcap recording.pcap

# Replay a ROS bag
python3 velodyne_decoder_final.py --bag data.bag

# Filter by range (1-30 meters only)
python3 velodyne_decoder_final.py --min-range 1.0 --max-range 30.0

# Filter by azimuth (front 180 degrees)
python3 velodyne_decoder_final.py --min-angle 0 --max-angle 180

# Adjust scan buffer (1 = sharpest, 5 = smoothest)
python3 velodyne_decoder_final.py --buffer-scans 1
```

## How It Works

```
VLP-16 Sensor
     |
     | UDP packets (port 2368, 1206 bytes each)
     v
[ velodyne_decoder StreamDecoder ]
     |
     | Structured NumPy arrays (x, y, z, intensity, ring, time)
     v
[ Scan Buffer (deque) ]
     |
     | Merged + filtered point cloud
     v
[ Open3D Visualizer ]
     |
     | Height-based RGB coloring
     v
  3D Render @ 15 FPS
```

The capture thread runs independently from the render loop. A thread-safe deque buffers complete 360-degree scans, and the visualizer merges and colors them at a capped frame rate.

## Integration

This decoder feeds directly into AVL's downstream perception stack:

- **Occupancy Grid Mapping** for obstacle detection
- **RTAB-Map SLAM** via ROS 2 Humble for autonomous navigation
- **Path Planning** (Pure Pursuit, DWA) on the AVL-002 go-kart
- **LeWitt-LiDAR** art visualization (see [lewitt-lidar](https://github.com/Ryan-Simpson/lewitt-lidar))

## Context

Built for the Autonomous Vehicle Laboratory at Cal Poly Pomona as part of ongoing research into LiDAR-based perception, SLAM, and digital twin reconstruction for autonomous vehicles.

## Author

**Ryan Simpson** - Research Assistant & Onboarding Director, Autonomous Vehicle Laboratory, Cal Poly Pomona

## License

MIT
