import argparse
import json
import logging
import os
import shutil
import tempfile
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import av
import av.bitstream
import numpy as np
from scipy.spatial.transform import Rotation
logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger("trinet_to_mcap")

# Note: foxglove.Imu is missing from the current foxglove_schemas_protobuf package.
# We will use foxglove.Vector3 to publish raw IMU data to separate topics for plotting.
from foxglove_schemas_protobuf.CameraCalibration_pb2 import CameraCalibration
from foxglove_schemas_protobuf.CompressedVideo_pb2 import CompressedVideo
from foxglove_schemas_protobuf.FrameTransform_pb2 import FrameTransform
from foxglove_schemas_protobuf.Quaternion_pb2 import Quaternion
from foxglove_schemas_protobuf.Vector3_pb2 import Vector3
from google.protobuf.timestamp_pb2 import Timestamp
from mcap_protobuf.writer import Writer

# Assuming trinet_tools is available locally
try:
    from trinet_tools import extract_sei, madgwick, reader
except ImportError:
    logger.error("trinet_tools not found. Please ensure it is in the current directory.")
    raise

def to_google_timestamp(nanos: int) -> Timestamp:
    ts = Timestamp()
    ts.seconds = nanos // 1_000_000_000
    ts.nanos = nanos % 1_000_000_000
    return ts

def matrix_to_quaternion(R: np.ndarray) -> Tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to a (x, y, z, w) quaternion using Scipy."""
    r = Rotation.from_matrix(R)
    # Scipy as_quat() returns [x, y, z, w]
    return tuple(r.as_quat())

def convert(mp4_path: Path, calib_path: Path, output_path: Path, use_mag: bool = True):
    with open(calib_path, 'r') as f:
        calib = json.load(f)

    with tempfile.TemporaryDirectory() as tmp_dir:
        logger.info(f"Extracting SEI data from {mp4_path} to {tmp_dir}...")
        # extract_sei.extract(mp4, out_dir) creates video.mp4, *.imu, *.vts
        extract_sei.extract(mp4_path, Path(tmp_dir))
        
        # Find extracted files. The tools might write to out_dir or CWD depending on implementation.
        def find_file(pattern, search_dir):
            matches = list(search_dir.glob(pattern))
            if not matches:
                # Fallback to current working directory
                matches = list(Path(".").glob(pattern))
            return matches[0] if matches else None

        imu_file = find_file("*.imu", Path(tmp_dir)) or find_file("imu.bin", Path(tmp_dir))
        vts_file = find_file("*.vts", Path(tmp_dir)) or find_file("frames.bin", Path(tmp_dir))
        clean_video = Path(tmp_dir) / "video.mp4"
        if not clean_video.exists():
            clean_video = Path("video.mp4")

        if not imu_file or not vts_file or not clean_video.exists():
            logger.error(f"Missing files: imu={imu_file}, vts={vts_file}, video={clean_video.exists()}")
            raise RuntimeError("Extraction failed: missing output files.")

        imu_data = reader.read_imu(str(imu_file))
        vts_data = reader.read_vts(str(vts_file))

        logger.info(f"Parsed {len(imu_data.accel)} IMU samples and {len(vts_data.sof_timestamps_ns)} frame timestamps.")
        
        # Compute Orientation (Head Pose) using Madgwick filter
        logger.info(f"Computing 3D orientation from IMU data ({'9-DOF' if use_mag else '6-DOF'})...")
        # madgwick.run_madgwick expects timestamps in seconds
        imu_timestamps_s = imu_data.timestamps_ns.astype(np.float64) / 1e9
        quat_xyzw = madgwick.run_madgwick(
            imu_data.accel, 
            imu_data.gyro, 
            imu_data.mag, 
            imu_timestamps_s,
            use_mag=use_mag,
            max_samples=200000 # Increase to avoid Slerp precision issues
        )

        # Synchronization parameters
        timeshift_cam_imu_sec = calib["extrinsics"]["timeshift_cam_imu_sec"]
        timeshift_ns = int(timeshift_cam_imu_sec * 1e9)
        
        # Clock alignment: t_imu = t_cam + timeshift_cam_imu_sec
        # So t_cam = t_imu - timeshift_ns
        # The sof_timestamps_ns is already in cam clock.
        # We need to express everything in a consistent clock. 
        # Let's use cam clock as the base for the MCAP.

        with open(output_path, "wb") as f_mcap:
            writer = Writer(f_mcap)
            
            # Store raw calibration as MCAP Metadata for archival/provenance
            logger.info("Writing raw calibration to MCAP metadata...")
            writer._writer.add_metadata(
                name="calibration_json",
                data={"content": json.dumps(calib, indent=2)}
            )
            
            # Static TF: imu -> cam0 (Extrinsics)
            R_cam_imu = np.array(calib["extrinsics"]["R_cam_imu"])
            t_cam_imu = np.array(calib["extrinsics"]["t_cam_imu_m"])
            
            qx, qy, qz, qw = matrix_to_quaternion(R_cam_imu)

            static_tf_msg = FrameTransform()
            static_tf_msg.parent_frame_id = "imu"
            static_tf_msg.child_frame_id = "cam0"
            static_tf_msg.translation.x = t_cam_imu[0]
            static_tf_msg.translation.y = t_cam_imu[1]
            static_tf_msg.translation.z = t_cam_imu[2]
            static_tf_msg.rotation.x = qx
            static_tf_msg.rotation.y = qy
            static_tf_msg.rotation.z = qz
            static_tf_msg.rotation.w = qw
            
            # Initial broadcast
            static_tf_msg.timestamp.CopyFrom(to_google_timestamp(vts_data.sof_timestamps_ns[0]))
            writer.write_message("/tf", static_tf_msg, log_time=vts_data.sof_timestamps_ns[0], publish_time=vts_data.sof_timestamps_ns[0])

            # Camera Calibration
            intr = calib["intrinsics"]
            calib_msg = CameraCalibration()
            calib_msg.timestamp.CopyFrom(to_google_timestamp(vts_data.sof_timestamps_ns[0]))
            calib_msg.frame_id = "cam0"
            calib_msg.width = intr["image_size"][0]
            calib_msg.height = intr["image_size"][1]
            calib_msg.distortion_model = "kannala_brandt" # Correct Foxglove standard name
            calib_msg.D.extend(intr["distortion"])
            calib_msg.K.extend([
                intr["fx"], 0, intr["cx"],
                0, intr["fy"], intr["cy"],
                0, 0, 1
            ])
            
            # Standard functional matrices for Foxglove/ROS visualization
            # R (Rectification) should be Identity for a single camera
            calib_msg.R.extend([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
            
            # P (Projection) should be [K | 0] for standard projection pipelines
            P_func = np.zeros((3, 4))
            P_func[0:3, 0:3] = np.array([
                [intr["fx"], 0, intr["cx"]],
                [0, intr["fy"], intr["cy"]],
                [0, 0, 1]
            ])
            calib_msg.P.extend(P_func.flatten().tolist())
            
            writer.write_message("/camera/calibration", calib_msg, log_time=vts_data.sof_timestamps_ns[0], publish_time=vts_data.sof_timestamps_ns[0])

            # Write IMU data
            gyro_bias = np.array([0.0, 0.0, 0.0])
            accel_bias = np.array([0.0, 0.0, 0.0])
            if calib["imu"].get("bias_source") == "kalibr_estimated":
                gyro_bias = np.array(calib["imu"]["gyro_bias_rad_s"])
                accel_bias = np.array(calib["imu"]["accel_bias_m_s2"])
                logger.info(f"Applying Kalibr estimated biases: gyro={gyro_bias}, accel={accel_bias}")

            for i in range(len(imu_data.timestamps_ns)):
                t_imu = int(imu_data.timestamps_ns[i])
                t_aligned = t_imu - timeshift_ns
                
                # Publish angular velocity
                gyro_msg = Vector3()
                gyro_msg.x = imu_data.gyro[i, 0] - gyro_bias[0]
                gyro_msg.y = imu_data.gyro[i, 1] - gyro_bias[1]
                gyro_msg.z = imu_data.gyro[i, 2] - gyro_bias[2]
                writer.write_message("/imu/angular_velocity", gyro_msg, log_time=t_aligned, publish_time=t_aligned)
                
                # Publish linear acceleration
                accel_msg = Vector3()
                accel_msg.x = imu_data.accel[i, 0] - accel_bias[0]
                accel_msg.y = imu_data.accel[i, 1] - accel_bias[1]
                accel_msg.z = imu_data.accel[i, 2] - accel_bias[2]
                writer.write_message("/imu/linear_acceleration", accel_msg, log_time=t_aligned, publish_time=t_aligned)

                # Publish raw magnetic field
                if imu_data.mag is not None:
                    mag_msg = Vector3()
                    mag_msg.x = imu_data.mag[i, 0]
                    mag_msg.y = imu_data.mag[i, 1]
                    mag_msg.z = imu_data.mag[i, 2]
                    writer.write_message("/imu/magnetic_field", mag_msg, log_time=t_aligned, publish_time=t_aligned)

                # Publish dynamic TF: world -> imu (Head Pose)
                head_tf = FrameTransform()
                head_tf.timestamp.CopyFrom(to_google_timestamp(t_aligned))
                head_tf.parent_frame_id = "world"
                head_tf.child_frame_id = "imu"
                # Orientation from Madgwick
                head_tf.rotation.x = quat_xyzw[i, 0]
                head_tf.rotation.y = quat_xyzw[i, 1]
                head_tf.rotation.z = quat_xyzw[i, 2]
                head_tf.rotation.w = quat_xyzw[i, 3]
                # Position is unknown (set to zero for egocentric rotation-only)
                writer.write_message("/tf", head_tf, log_time=t_aligned, publish_time=t_aligned)

            # Transcode video to remove B-frames (Foxglove compatibility requirement)
            logger.info("Transcoding video to remove B-frames (Foxglove compatibility)...")
            import subprocess
            nobf_video = Path(tmp_dir) / "video_nobf.mp4"
            subprocess.run([
                "ffmpeg", "-y", "-i", str(clean_video),
                "-c:v", "libx264", "-preset", "ultrafast", "-bf", "0",
                "-crf", "23", str(nobf_video)
            ], check=True, capture_output=True)
            clean_video = nobf_video

            # Write Video Frames
            logger.info("Extracting H.264 packets from video and converting to Annex B...")
            container = av.open(str(clean_video))
            stream = container.streams.video[0]
            
            # Foxglove requires Annex B format for H.264
            bsf = None
            if stream.codec_context.name == 'h264':
                bsf = av.bitstream.BitStreamFilterContext('h264_mp4toannexb', stream)

            frame_idx = 0
            for packet in container.demux(stream):
                if packet.dts is None:
                    continue
                
                # Apply bitstream filter to convert AVCC to Annex B
                packets_to_write = [packet]
                if bsf:
                    packets_to_write = bsf.filter(packet)

                for p in packets_to_write:
                    # Use sof_timestamps_ns from vts_data
                    if frame_idx >= len(vts_data.sof_timestamps_ns):
                        logger.warning(f"More video packets than timestamps ({frame_idx} >= {len(vts_data.sof_timestamps_ns)})")
                        break
                    
                    t_frame = vts_data.sof_timestamps_ns[frame_idx]
                    
                    # Continuous TF Broadcast: imu -> cam0 (ensure persistence in Foxglove)
                    static_tf_msg.timestamp.CopyFrom(to_google_timestamp(t_frame))
                    writer.write_message("/tf", static_tf_msg, log_time=t_frame, publish_time=t_frame)

                    vid_msg = CompressedVideo()
                    vid_msg.timestamp.CopyFrom(to_google_timestamp(t_frame))
                    vid_msg.frame_id = "cam0"
                    vid_msg.data = bytes(p)
                    vid_msg.format = "h264"
                    
                    writer.write_message("/camera/image/compressed", vid_msg, log_time=t_frame, publish_time=t_frame)
                    frame_idx += 1
            
            # Flush the filter if needed
            if bsf:
                for p in bsf.filter(None):
                    if frame_idx < len(vts_data.sof_timestamps_ns):
                        t_frame = vts_data.sof_timestamps_ns[frame_idx]
                        
                        # Continuous TF Broadcast for flushed frames
                        static_tf_msg.timestamp.CopyFrom(to_google_timestamp(t_frame))
                        writer.write_message("/tf", static_tf_msg, log_time=t_frame, publish_time=t_frame)

                        vid_msg = CompressedVideo()
                        vid_msg.timestamp.CopyFrom(to_google_timestamp(t_frame))
                        vid_msg.frame_id = "cam0"
                        vid_msg.data = bytes(p)
                        vid_msg.format = "h264"
                        writer.write_message("/camera/image/compressed", vid_msg, log_time=t_frame, publish_time=t_frame)
                        frame_idx += 1
            
            writer.finish()
            logger.info(f"Successfully wrote MCAP to {output_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert Trinet MP4 to MCAP")
    parser.add_argument("--input", type=Path, required=True, help="Input recording.mp4")
    parser.add_argument("--calibration", type=Path, required=True, help="Input calibration.json")
    parser.add_argument("--output", type=Path, default=Path("output.mcap"), help="Output MCAP file")
    
    # Magnetometer options
    parser.add_argument("--use-mag", action="store_true", default=True, help="Enable 9-DOF fusion (default)")
    parser.add_argument("--no-mag", action="store_false", dest="use_mag", help="Disable magnetometer (use 6-DOF)")
    
    args = parser.parse_args()
    
    convert(args.input, args.calibration, args.output, args.use_mag)
