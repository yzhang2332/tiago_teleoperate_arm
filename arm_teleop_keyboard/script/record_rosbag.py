import subprocess
import datetime
import os
import time
import signal

# Define a signal handler for SIGINT
def signal_handler(sig, frame):
    print('Stopping recording and throttling...')
    os.killpg(os.getpgid(process.pid), signal.SIGINT)
    os.killpg(os.getpgid(throttle_rgb_process.pid), signal.SIGINT)
    os.killpg(os.getpgid(audio_recording.pid), signal.SIGINT)
    process.wait()
    throttle_rgb_process.wait()
    audio_recording.wait()
    print('Recording and throttling stopped.')
    exit(0)

# Register the signal handler for SIGINT (Ctrl+C)
signal.signal(signal.SIGINT, signal_handler)

# Ensure the directory exists
directory = "/home/pal/rosbags/"
# directory = "/home/yanzhang/pal/rosbags/"
if not os.path.exists(directory):
    os.makedirs(directory)

# Format the current date and time as a string
current_time = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")


# Define the bag file name using the current date and time
# bag_file_name = f"/home/rosbags/record_{current_time}.bag"
bag_file_name = f"{directory}record_{current_time}.bag"

# Throttle command for the camera topic
throttle_rgb_command = "rosrun topic_tools throttle messages /xtion/rgb/image_raw 4.0 /xtion/rgb/image_raw_throttled"
throttle_rgb_process = subprocess.Popen(throttle_rgb_command, shell=True, preexec_fn=os.setsid)


throttle_dep_command = "rosrun topic_tools throttle messages /xtion/depth_registered/image_raw 4.0 /xtion/depth/image_raw_throttled"

# throttle_dep_command = "rosrun topic_tools throttle messages /xtion/depth/image_raw 4.0 /xtion/depth/image_raw_throttled"
throttle_dep_process = subprocess.Popen(throttle_dep_command, shell=True, preexec_fn=os.setsid)

# Start audio recording
audio_command = "rosrun audio_capture audio_capture _device:=hw:1,0"
audio_recording = subprocess.Popen(audio_command, shell=True, preexec_fn=os.setsid)


# Adjusted topics with throttled camera topic
topics = "/xtion/rgb/image_raw_throttled /xtion/depth/image_raw_throttled /joint_states /audio /aruco_pose_tf /arm_cartesian"

# Define the command to start recording all topics to the named bag file
command = f"rosbag record --lz4 {topics} -O {bag_file_name}"
# Start recording
process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)

# Keep the script running until Ctrl+C is pressed
print("Recording... Press Ctrl+C to stop.")
while True:
    time.sleep(1)
