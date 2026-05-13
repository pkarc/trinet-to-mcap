# Trinet to MCAP Converter

[![Language: Spanish](https://img.shields.io/badge/Language-Spanish-red.svg)](README.es.md)

A standalone Python application to convert egocentric Trinet camera recordings (MP4 with embedded SEI IMU data) into LZ4-compressed MCAP files compatible with Foxglove Studio.

## Features
- **Data Extraction**: Automatically splits Trinet MP4 files into clean video and binary IMU/VTS sidecars.
- **Annex B Conversion**: Converts H.264 stream from AVCC (MP4) to Annex B format for seamless decoding in Foxglove.
- **3D Head Pose**: Computes real-time orientation using the Madgwick filter (supports both 6-DOF and 9-DOF).
- **Robotics Standards**:
    - **TF Tree**: `world` -> `imu` -> `cam0`.
    - **Intrinsics**: Published via `foxglove.CameraCalibration` (Kannala-Brandt model).
    - **IMU**: High-frequency data for angular velocity, linear acceleration, and magnetic field.

## Prerequisites
- Python 3.8+
- `ffmpeg` and `ffprobe` installed in your system PATH.

## Installation
1. Activate your virtual environment (ensure it's sourced).
2. Install the required dependencies:
   ```bash
   pip install -r requirements.txt
   pip install ahrs scipy
   ```

## Usage
Run the conversion script with the following arguments:
```bash
python convert.py --input sample_data/clothes.mp4 --calibration sample_data/calibration.json --output sample_output/output.mcap [OPTIONS]
```

**Options:**
- `--use-mag`: Enable 9-DOF fusion using the magnetometer (Default).
- `--no-mag`: Disable magnetometer for pose estimation (uses 6-DOF fusion).

## Data Topics
| Topic | Schema | Description |
| --- | --- | --- |
| `/camera/image/compressed` | `foxglove.CompressedVideo` | H.264 video in Annex B format. |
| `/camera/calibration` | `foxglove.CameraCalibration` | Fisheye intrinsics (Kannala-Brandt). |
| `/imu/angular_velocity` | `foxglove.Vector3` | Corrected gyroscope data. |
| `/imu/linear_acceleration` | `foxglove.Vector3` | Corrected accelerometer data. |
| `/imu/magnetic_field` | `foxglove.Vector3` | Raw magnetometer data. |
| `/tf` | `foxglove.FrameTransform` | Dynamic (`world->imu`) and Static (`imu->cam0`) transforms. |

## Extrinsics and TF Tree
The converter builds a hierarchical coordinate system (TF Tree) to represent the camera's movement in 3D space:
1. **`world` -> `imu`**: A dynamic transform representing the head pose, calculated using the Madgwick sensor fusion filter.
2. **`imu` -> `cam0`**: A **static transform using your extrinsics**. The values `R_cam_imu` (Rotation) and `t_cam_imu_m` (Translation) from `calibration.json` are embedded here to define the exact physical offset between the IMU and the camera lens.

In **Foxglove Studio**, you can visualize this by setting the "Global frame" to `world` in a 3D panel and enabling the "Transforms" layer.

## Archival and Metadata
To ensure data reproducibility and provenance, the entire content of the input `calibration.json` is stored as **global MCAP metadata** within the file.
- **Metadata Name**: `calibration_json`
- **Description**: Contains the raw JSON string with original extrinsics (`R_cam_imu`, `t_cam_imu_m`) and intrinsics before any processing.
