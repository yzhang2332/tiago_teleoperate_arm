import tkinter as tk
import random
import signal
#Installed to play audio without ROS
import pygame  

# Init pygame mixer
pygame.mixer.init()

voice_version = 1
toggle_button = None

# Try importing ROS packages, but allow running without them
try:
    import rospy
    import actionlib
    from std_msgs.msg import String
    from pal_interaction_msgs.msg import TtsAction, TtsGoal
    ROS_ENABLED = True
except ImportError:
    print("ROS not detected. Running in GUI test mode.")
    ROS_ENABLED = False

# Global flag to indicate shutdown
shutdown_flag = False

def signal_handler(sig, frame):
    global shutdown_flag
    print('Shutting down...')
    shutdown_flag = True  # Set the flag to indicate shutdown

def button_clicked(audio_file, publisher=None):
    """Simulated ROS publishing or local print."""
    if ROS_ENABLED:
        rospy.loginfo(f"Publishing message to /play_audio: {audio_file}")
        publisher.publish(audio_file)
    else:
        print(f"[TEST MODE] Would publish: {audio_file}")

def play_audio(audio_file):
    global voice_version
    versioned_file = audio_file.replace(".mp3", f"_v{voice_version}.mp3")
    pygame.mixer.music.stop()
    pygame.mixer.music.load(versioned_file)
    pygame.mixer.music.play()        

def create_single_random_button(master, audio_files, publisher=None):
    """Create a button that plays a random audio file."""
    def on_click():
        chosen_audio = random.choice(audio_files)
        button_clicked(chosen_audio, publisher)
        
    button = tk.Button(master, text="Play Random Audio", command=on_click, font=("Helvetica", 12))
    button.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5, anchor='w')

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
    global root, message_entry

    if ROS_ENABLED:
        # Initialize ROS node
        rospy.init_node("gui", anonymous=True)

        # Create ROS publisher for /play_audio
        audio_publisher = rospy.Publisher('/play_audio', String, queue_size=10)

        # Set up TTS action client
        tts_client = actionlib.SimpleActionClient('/tts', TtsAction)
        tts_client.wait_for_server()
        rospy.loginfo("TTS server connected.")
    else:
        audio_publisher = None
        tts_client = None

    # Initialize GUI
    root = tk.Tk()
    root.title("Voice Response GUI")
    root.geometry("400x650") # Setting window size

    # Create Sections
    position_frame = tk.Frame(root)
    position_frame.pack(pady=10)

    verb_frame = tk.Frame(root)
    verb_frame.pack(pady=10)

    color_frame = tk.Frame(root)
    color_frame.pack(pady=10)

    number_frame = tk.Frame(root)
    number_frame.pack(pady=10)

    # Voice Toggle Button
    toggle_button = tk.Button(root, text=f"Voice v{voice_version}", font=("Helvetica", 14), command=toggle_voice)
    toggle_button.pack(pady=10)

    # Position Section
    position_label = tk.Label(position_frame, text="Position", font=("Helvetica", 14, "bold"))
    position_label.pack()

    position_buttons = []
    for i in range(1, 6):
        btn = tk.Button(position_frame, text=str(i), font=("Helvetica", 14), width=5, height=2,
                        command=lambda i=i: play_audio(f"tile{i}.mp3"))
        btn.pack(side=tk.LEFT, padx=5)
        position_buttons.append(btn)

    # Upper & Lower Buttons under each position button
    #for i in range(5):
        frame = tk.Frame(position_frame)
        frame.pack(side=tk.LEFT, padx=5)
        upper_btn = tk.Button(frame, text="U", font=("Helvetica", 10), width=2, height=1,
                            command=lambda: play_audio("upper.mp3"))
        lower_btn = tk.Button(frame, text="L", font=("Helvetica", 10), width=2, height=1,
                            command=lambda: play_audio("lower.mp3"))
        upper_btn.pack()
        lower_btn.pack()

    # Verb Section
    verb_buttons = [
        ("is", "is.mp3"),
        ("are", "are.mp3")
    ]
    for text, audio in verb_buttons:
        btn = tk.Button(verb_frame, text=text, font=("Helvetica", 14), width=5, height=2,
                        command=lambda audio=audio: play_audio(audio))
        btn.pack(side=tk.LEFT, padx=10)

    # Color Section
    color_label = tk.Label(color_frame, text="Color", font=("Helvetica", 14, "bold"))
    color_label.pack()

    colors = {
        "Blue": "blue",
        "Black": "black",
        "Red": "red",
        "Green": "green"
    }

    for color_text, color_hex in colors.items():
        btn = tk.Button(color_frame, text=color_text, font=("Helvetica", 14), fg=color_hex, width=10, height=2,
                        command=lambda color=color_text: play_audio(f"{color.lower()}.mp3"))
        btn.pack(side=tk.LEFT, padx=5)

    # Number Section
    number_label = tk.Label(number_frame, text="Number", font=("Helvetica", 14, "bold"))
    number_label.pack()

    for i in range(1, 6):
        btn = tk.Button(number_frame, text=str(i), font=("Helvetica", 14), width=5, height=2,
                        command=lambda i=i: play_audio(f"number_{i}.mp3"))
        btn.pack(side=tk.LEFT, padx=5)
    # Create main container
    # main_container = tk.PanedWindow(root, orient='vertical')
    # main_container.pack(fill=tk.BOTH, expand=True)

    # Create different sections
    # one_frame = tk.Frame(main_container, background='lightgray', height=200)
    # two_frame = tk.Frame(main_container, background='darkgray', height=200)
    # text_input_frame = tk.Frame(main_container, background='darkgrey', height=400)

    # main_container.add(one_frame)
    #main_container.add(two_frame)
    #main_container.add(text_input_frame)

    # Add labels
    #one_label = tk.Label(one_frame, text="Play Random Audio", bg='lightgray')
    #two_label = tk.Label(two_frame, text="Play Specific Audio", bg='darkgray')
    #text_input_label = tk.Label(text_input_frame, text="Type your message (TTS):", bg='darkgray')

    #one_label.pack(side=tk.TOP, anchor='nw') 
    #two_label.pack(side=tk.TOP, anchor='nw')
    #text_input_label.pack(side=tk.TOP, anchor='nw')

    # Define audio files (simulated)
    #audio_files = ["../Audios/this_tile_V1.mp3", "../Audios/no.wav"]
    
    # Create buttons to simulate publishing audio messages
    #create_single_random_button(one_frame, audio_files, audio_publisher)
    #create_buttons(two_frame, audio_files, audio_publisher)

    # Create an input box for TTS
    #message_entry = tk.Entry(text_input_frame, font=("Helvetica", 12), width=50)
    #message_entry.pack(side=tk.TOP, padx=5, pady=5)

    #def handle_text_input(event):
    #    typed_text = message_entry.get()
    #    if typed_text.strip():
    #        send_text_to_speech(typed_text, tts_client)
    #    message_entry.delete(0, tk.END)

    # Bind Enter key to process text input (TTS)
    #message_entry.bind('<Return>', handle_text_input)

    # Start GUI event loop
    root.mainloop()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)  # Register signal handler
    gui_main()
