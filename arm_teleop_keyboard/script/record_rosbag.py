import subprocess
import datetime
import os
import time
import signal

# Ensure the directory exists
directory = "/home/pal/rosbags/"
if not os.path.exists(directory):
    os.makedirs(directory)

# Format the current date and time as a string
current_time = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")


# Define the bag file name using the current date and time
# bag_file_name = f"/home/rosbags/record_{current_time}.bag"
bag_file_name = f"{directory}record_{current_time}.bag"

topics = "/xtion/rgb/image_raw /joint_states /audio"

# Define the command to start recording all topics to the named bag file
command = f"rosbag record {topics} -O {bag_file_name}"
# Start recording
process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)

# Wait for a while or execute other logic
time.sleep(10)

# Stop recording by sending SIGINT to the process group, emulating Ctrl+C
os.killpg(os.getpgid(process.pid), signal.SIGINT)

# Optionally, wait for the command to complete
process.wait()
