import tkinter as tk
import random
import signal
import sys
import os
#Installed to play audio without ROS
# import pygame  

# Init pygame mixer
# pygame.mixer.init()

voice_version = 1
toggle_button = None
root = None

# Try importing ROS packages, but allow running without them
try:
    import rospy
    import actionlib
    from std_msgs.msg import String
    from pal_interaction_msgs.msg import TtsAction, TtsGoal
    import paramiko
    import threading
    ROS_ENABLED = True
except ImportError:
    print("ROS not detected. Running in GUI test mode.")
    ROS_ENABLED = False

# Global flag to indicate shutdown
shutdown_flag = False

def signal_handler(sig, frame):
    global shutdown_flag, root
    print('Shutting down...')
    shutdown_flag = True
    if root:
        root.quit()
    if ROS_ENABLED:
        rospy.signal_shutdown("Manual shutdown")
    

def button_clicked(audio_file):
    """Publish the audio file name to ROS."""
    # versioned_file = audio_file.replace(".mp3", f"_v{voice_version}.mp3")
    versioned_file = f"../hanabi_audios/{audio_file.replace('.mp3', f'_v{voice_version}.wav')}"

    # script_dir = os.path.dirname(os.path.abspath(__file__))  # Absolute dir of the script
    # audio_filename = audio_file.replace('.mp3', f'_v{voice_version}.wav')
    # versioned_file = os.path.join(script_dir, 'hanabi_audios', audio_filename)
    
    # versioned_file = f"home/pal/tiago_teleoperate_arm/arm_teleop_keyboard/script/hanabi_audios/{audio_file.replace('.mp3', f'_v{voice_version}.wav')}"
    if ROS_ENABLED and publisher:
        rospy.loginfo(f"Publishing message to /play_audio: {versioned_file}")
        publisher.publish(versioned_file)
    else:
        print(f"[TEST MODE] Would publish: {versioned_file}")
     

def create_buttons(master, audio_files, publisher=None):
    """Create buttons for each available audio file."""
    for audio_file in audio_files:
        button = tk.Button(master, text=f"Play {audio_file}", command=lambda a=audio_file: button_clicked(a, publisher), font=("Helvetica", 12))
        button.pack(side=tk.TOP, fill=tk.X, padx=2, pady=2, anchor='w')

def send_text_to_speech(text, tts_client=None):
    """Simulated TTS function for GUI testing."""
    if ROS_ENABLED:
        rospy.loginfo("Sending text to TTS!")
        goal = TtsGoal()
        goal.rawtext.text = text
        goal.rawtext.lang_id = "en_GB"
        tts_client.send_goal_and_wait(goal)
        rospy.loginfo(f"TTS Spoke: {text}")
    else:
        print(f"[TEST MODE] Would send to TTS: {text}")

def toggle_voice():
    """Toggle between voice version 1 and 2."""
    global voice_version, toggle_button
    voice_version = 2 if voice_version == 1 else 1
    if toggle_button:
            toggle_button.config(text=f"Voice V{voice_version}")

def gui_main():
    global publisher, tts_client, toggle_button, root

    if ROS_ENABLED:
        rospy.init_node("gui_ros_audio_publisher", anonymous=True)
        publisher = rospy.Publisher('/play_audio', String, queue_size=10)
        tts_client = actionlib.SimpleActionClient('/tts', TtsAction)
        tts_client.wait_for_server()
        rospy.loginfo("TTS server connected.")

    root = tk.Tk()
    root.title("Voice Response GUI")
    root.geometry("500x580")

    position_frame = tk.Frame(root)
    verb_frame = tk.Frame(root)
    color_frame = tk.Frame(root)
    number_frame = tk.Frame(root)
    action_frame = tk.Frame(root)
    position_frame.pack(pady=10)
    verb_frame.pack(pady=10)
    color_frame.pack(pady=10)
    number_frame.pack(pady=10)
    action_frame.pack(pady=10)

    toggle_button = tk.Button(root, text=f"Voice v{voice_version}", font=("Helvetica", 14), command=toggle_voice)
    toggle_button.pack(pady=10)

    # Position buttons
    position_label = tk.Label(position_frame, text="Position", font=("Helvetica", 14, "bold"))
    position_label.pack()
    for i in range(1, 6):
        btn = tk.Button(position_frame, text=str(i), font=("Helvetica", 14), width=5, height=2,
                        command=lambda i=i: button_clicked(f"tile{i}.mp3"))
        btn.pack(side=tk.LEFT, padx=5)

    # Upper/Lower buttons
    for i in range(5):
        frame = tk.Frame(position_frame)
        frame.pack(side=tk.LEFT, padx=5)
        upper_btn = tk.Button(frame, text="U", font=("Helvetica", 10), width=2, height=1,
                              command=lambda: button_clicked("upper.mp3"))
        lower_btn = tk.Button(frame, text="L", font=("Helvetica", 10), width=2, height=1,
                              command=lambda: button_clicked("lower.mp3"))
        upper_btn.pack()
        lower_btn.pack()


    # Verb section
    for text, audio in [("is", "is.mp3"), ("are", "are.mp3")]:
        btn = tk.Button(verb_frame, text=text, font=("Helvetica", 14), width=5, height=2,
                        command=lambda audio=audio: button_clicked(audio))
        btn.pack(side=tk.LEFT, padx=10)

    # Colour section
    color_label = tk.Label(color_frame, text="Color", font=("Helvetica", 14, "bold"))
    color_label.pack()
    colors = {"Blue": "blue", "Black": "black", "Red": "red", "Green": "green"}
    for color_text, color_hex in colors.items():
        btn = tk.Button(color_frame, text=color_text, font=("Helvetica", 14), fg=color_hex, width=10, height=2,
                        command=lambda color=color_text: button_clicked(f"{color.lower()}.mp3"))
        btn.pack(side=tk.LEFT, padx=5)

    # Number section
    number_label = tk.Label(number_frame, text="Number", font=("Helvetica", 14, "bold"))
    number_label.pack()
    for i in range(1, 6):
        btn = tk.Button(number_frame, text=str(i), font=("Helvetica", 14), width=5, height=2,
                        command=lambda i=i: button_clicked(f"number_{i}.mp3"))
        btn.pack(side=tk.LEFT, padx=5)

    # Communicate section
    action_label = tk.Label(action_frame, text="Communicate", font=("Helvetica", 14, "bold"))
    # action_label.pack()
    action_label.grid(row=0, column=0, columnspan=2, pady=(0, 5))


    actions = [
        ("I'll go first", "goFirst.mp3"), 
        ("I'm gonna give a clue", "giveClue.mp3"),
        ("I'm gonna discard a tile", "discardTile.mp3"),
        ("I'm gonna play a tile", "playTile.mp3"),
        ("Can you move the clue token?", "clueToken.mp3")

    ]


    for idx, (label, audio_file) in enumerate(actions):
        row = 1 + idx // 2
        col = idx % 2
        btn = tk.Button(action_frame, text=label, font=("Helvetica", 12), width=25, height=2,
                        command=lambda audio_file=audio_file: button_clicked(audio_file))
        btn.grid(row=row, column=col, padx=5, pady=5)

    def check_shutdown():
        """Check if the shutdown flag has been set and close the application if so."""
        if shutdown_flag:
            root.destroy()  # Close the Tkinter window
            rospy.signal_shutdown('Ctrl+C pressed')  # Shutdown ROS node
        else:
            root.after(100, check_shutdown)
    root.after(100, check_shutdown)

    root.mainloop()

def start_remote_script():
    hostname = "tiago-196c"
    port = 22
    username = "pal"
    password = "pal"  # Replace with actual password

    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(hostname, port=port, username=username, password=password)

        command = "source /opt/ros/noetic/setup.bash && cd scripts && python3 Audio.py"
        stdin, stdout, stderr = ssh.exec_command(command)

        print("STDOUT:")
        print(stdout.read().decode())

        print("STDERR:")
        print(stderr.read().decode())

        ssh.close()
    except Exception as e:
        print(f"SSH connection failed: {e}")


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    # Start the remote SSH process in a separate thread
    ssh_thread = threading.Thread(target=start_remote_script, daemon=True)
    ssh_thread.start()

    # Run the GUI
    gui_main()