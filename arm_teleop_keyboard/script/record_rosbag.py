import subprocess
import datetime

# Format the current date and time as a string
current_time = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
print(current_time)

# Define the bag file name using the current date and time
bag_file_name = f"/home/rosbags/record_{current_time}.bag"
topics = "/xtion/rgb/image_raw /joint_states /audio"

# Define the command to start recording all topics to the named bag file
command = f"rosbag record -a -O {bag_file_name}"
# Start recording
process = subprocess.Popen(command, shell=True)

# Wait for a while or execute other logic
# Example: sleep for 10 seconds
import time
time.sleep(10)

# Stop recording by terminating the subprocess
process.terminate()

# Optionally, wait for the command to complete
process.wait()
