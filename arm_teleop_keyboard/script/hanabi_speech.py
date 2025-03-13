import tkinter as tk
import rospy
import actionlib
import subprocess
import random
import signal
from std_msgs.msg import String
from pal_interaction_msgs.msg import TtsAction, TtsGoal

# Global flag to indicate shutdown
shutdown_flag = False

def signal_handler(sig, frame):
    global shutdown_flag
    print('Shutting down...')
    shutdown_flag = True  # Set the flag to indicate shutdown

def button_clicked(audio_file, publisher):
    """Publish the selected audio file name to the /play_audio topic."""
    rospy.loginfo(f"Publishing message to /play_audio: {audio_file}")
    publisher.publish(audio_file)

def create_single_random_button(master, audio_files, publisher):
    """Create a button that plays a random audio file."""
    def on_click():
        chosen_audio = random.choice(audio_files)
        button_clicked(chosen_audio, publisher)
        
    button = tk.Button(master, text="Play Random Audio", command=on_click, font=("Helvetica", 12))
    button.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5, anchor='w')

def create_buttons(master, audio_files, publisher):
    """Create buttons for each available audio file."""
    for audio_file in audio_files:
        button = tk.Button(master, text=f"Play {audio_file}", command=lambda a=audio_file: button_clicked(a, publisher), font=("Helvetica", 12))
        button.pack(side=tk.TOP, fill=tk.X, padx=2, pady=2, anchor='w')

def send_text_to_speech(text, tts_client):
    """Send the typed text to the TTS action server."""
    rospy.loginfo("Sending text to TTS!")
    goal = TtsGoal()
    goal.rawtext.text = text
    goal.rawtext.lang_id = "en_GB"
    tts_client.send_goal_and_wait(goal)
    rospy.loginfo(f"TTS Spoke: {text}")

def gui_main():
    global root, message_entry

    # Initialize ROS node
    rospy.init_node("gui", anonymous=True)

    # Create ROS publisher for /play_audio
    audio_publisher = rospy.Publisher('/play_audio', String, queue_size=10)

    # Set up TTS action client
    tts_client = actionlib.SimpleActionClient('/tts', TtsAction)
    tts_client.wait_for_server()
    rospy.loginfo("TTS server connected.")

    # Initialize GUI
    root = tk.Tk()
    root.title("Voice Response GUI")
    
    # Create main container
    main_container = tk.PanedWindow(root, orient='vertical')
    main_container.pack(fill=tk.BOTH, expand=True)

    # Create different sections
    one_frame = tk.Frame(main_container, background='lightgray', height=200)
    two_frame = tk.Frame(main_container, background='darkgray', height=200)
    text_input_frame = tk.Frame(main_container, background='darkgrey', height=400)

    main_container.add(one_frame)
    main_container.add(two_frame)
    main_container.add(text_input_frame)

    # Add labels
    one_label = tk.Label(one_frame, text="Play Random Audio", bg='lightgray')
    two_label = tk.Label(two_frame, text="Play Specific Audio", bg='darkgray')
    text_input_label = tk.Label(text_input_frame, text="Type your message (TTS):", bg='darkgray')

    one_label.pack(side=tk.TOP, anchor='nw') 
    two_label.pack(side=tk.TOP, anchor='nw')
    text_input_label.pack(side=tk.TOP, anchor='nw')

    # Define audio files
    audio_files = ["../Audios/RobotTurn.wav", "../Audios/no.wav"]
    
    # Create buttons to publish audio file messages
    create_single_random_button(one_frame, audio_files, audio_publisher)
    create_buttons(two_frame, audio_files, audio_publisher)

    # Create an input box for TTS
    message_entry = tk.Entry(text_input_frame, font=("Helvetica", 12), width=50)
    message_entry.pack(side=tk.TOP, padx=5, pady=5)

    def handle_text_input(event):
        typed_text = message_entry.get()
        if typed_text.strip():
            send_text_to_speech(typed_text, tts_client)
        message_entry.delete(0, tk.END)

    # Bind Enter key to process text input (TTS)
    message_entry.bind('<Return>', handle_text_input)

    # Start GUI event loop
    root.mainloop()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)  # Register signal handler
    gui_main()
