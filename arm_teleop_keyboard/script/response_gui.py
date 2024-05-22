import tkinter as tk
import rospy
import actionlib
from pal_interaction_msgs.msg import TtsAction, TtsGoal
import random
import signal 

# Global flag to indicate shutdown
shutdown_flag = False

def signal_handler(sig, frame):
    global shutdown_flag
    print('Shutting down...')
    shutdown_flag = True  # Set the flag to indicate shutdown


def button_clicked(button_text, tts_client):
    rospy.loginfo("Inside the tts function!!!")
    
    ######
    # Create a goal to say our sentence
    goal = TtsGoal()
    goal.rawtext.text = button_text
    goal.rawtext.lang_id = "en_GB"
    # Send the goal and wait
    tts_client.send_goal_and_wait(goal)
    #####

    rospy.loginfo(f"{button_text} button clicked!")

def create_single_random_button(master, words, tts_client):
    """Create a single button that, when clicked, randomly chooses a sentence to say."""
    def on_click():
        # Randomly choose one of the words
        chosen_word = random.choice(words)
        # Pass the chosen word to the button_clicked function
        button_clicked(chosen_word, tts_client)
        
    button = tk.Button(master, text=words[0], command=on_click, font=("Helvetica", 12))
    button.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5, anchor='w')


def create_buttons_intro(master, words, tts_client):
    for word in words:
        button = tk.Button(master, text=word, command=lambda w=word: button_clicked(w, tts_client), font=("Helvetica", 10))
        button.pack(side=tk.TOP, fill=tk.X, padx=2, pady=2, anchor='w')


def create_buttons(master, words, tts_client):
    """Create buttons in the specified master widget based on the words provided."""
    for word in words:
        button = tk.Button(master, text=word, command=lambda w=word: button_clicked(w, tts_client), font=("Helvetica", 12))
        # button.pack(side=tk.LEFT, padx=5, pady=5)
        button.pack(side=tk.TOP, fill=tk.X, padx=2, pady=2, anchor='w')

def create_buttons_multi_row(master, words, tts_client):
    button_frame = tk.Frame(master)  # Create a new frame within the master to hold the buttons
    button_frame.pack(fill=tk.X, padx=2, pady=2)  # Pack this frame into the master

    row, col = 0, 0  # Starting row and column
    max_per_row = 3  # Maximum number of buttons per row

    for index, word in enumerate(words):
        button = tk.Button(button_frame, text=word, command=lambda w=word: button_clicked(w, tts_client),
                           font=("Helvetica", 12))
        button.grid(row=row, column=col, padx=2, pady=2, sticky="ew")

        col += 1  # Move to the next column
        if (index + 1) % max_per_row == 0:
            # If max_per_row is reached, reset column and move to the next row
            row += 1
            col = 0

    # Configure the columns to have the same width
    for i in range(max_per_row):
        button_frame.grid_columnconfigure(i, weight=1)
    

def gui_main():
    global root

    # Initialize the main window
    root = tk.Tk()
    root.title("Voice Response GUI")
    
    ######
    tts_client = actionlib.SimpleActionClient('/tts', TtsAction)
    tts_client.wait_for_server()
    rospy.loginfo("Tts server connected.")
    # tts_client = False  
    ######
    
                                                                                    
    # Create the main container
    main_container = tk.PanedWindow(root, orient='vertical')
    main_container.pack(fill=tk.BOTH, expand=True)

    # Create two frames (sessions)
    intro_frame = tk.Frame(main_container, background='darkgray', height=800)
    one_frame = tk.Frame(main_container, background='lightgray', height=200)
    two_frame = tk.Frame(main_container, background='darkgray', height=200)
    three_frame = tk.Frame(main_container, background='lightgray', height=200)
    four_frame = tk.Frame(main_container, background='darkgray', height=200)
    five_frame = tk.Frame(main_container, background='lightgray', height=200)
    text_input_frame = tk.Frame(main_container, background='darkgrey', height=400)


    main_container.add(intro_frame)
    main_container.add(one_frame)
    main_container.add(two_frame)
    main_container.add(three_frame)
    main_container.add(four_frame)
    main_container.add(five_frame)
    main_container.add(text_input_frame)


    # Add a text description (label) to each frame
    intro_label = tk.Label(intro_frame, text="Introduction", bg='darkgray')
    one_label = tk.Label(one_frame, text="Direct/ISA Group", bg='lightgray')
    two_label = tk.Label(two_frame, text="Can you?", bg='darkgray')
    three_label = tk.Label(three_frame, text="Provide information", bg='lightgray')
    four_label = tk.Label(four_frame, text="Missed", bg='darkgray')
    five_label = tk.Label(five_frame, text="Help and others", bg='lightgray')
    text_input_label = tk.Label(text_input_frame, text="Type your message:", bg='darkgray')

    # Pack the labels to the top of each frame
    intro_label.pack(side=tk.TOP, anchor='nw') 
    one_label.pack(side=tk.TOP, anchor='nw') 
    two_label.pack(side=tk.TOP, anchor='nw')
    three_label.pack(side=tk.TOP, anchor='nw')
    four_label.pack(side=tk.TOP, anchor='nw')
    five_label.pack(side=tk.TOP, anchor='nw')
    text_input_label.pack(side=tk.TOP, anchor='nw')

    # Define the words for the buttons in both sessions
    intro_session_words = ["Hey, Mate! How are you? I’m so excited to work with you.", "How can I help you?", "We did a great job. Now what else are we going to do?", "I’m sorry, that’s too much detail to remember. \nLet’s get hands on the work, then you can fill me in when it’s need."]
    one_session_words = ["Sure.", "Yes, I can.", "I will work on that.", "Yes, of course."]
    two_session_words = ["Yes, I'm able to do that.", "Yes, I will do it if you want."]
    three_session_words = ["I'm happy to hear that.", "It's good to know."]
    four_session_words = ["I'm sorry I don't understand.", "Sorry I missed that. Can you say it again?"]
    five_session_words = ["I need help.", "I can't reach that.", "What should I do next?", "Thank you.", "You're welcome.", "Yes.", "No.", "Done.", "Sorry, I can't."]

    # Validation function to allow only specific characters
    def validate_input(char):
        allowed_characters = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ,.?! "
        return char in allowed_characters

    # Register the validation function
    validate_command = root.register(validate_input)

    # Entry widget for typing the message
    message_entry = tk.Entry(text_input_frame, font=("Helvetica", 12), width=50)
    message_entry.pack(side=tk.TOP, padx=5, pady=5)   

    def check_shutdown():
        """Check if the shutdown flag has been set and close the application if so."""
        if shutdown_flag:
            root.destroy()  # Close the Tkinter window
            rospy.signal_shutdown('Ctrl+C pressed')  # Shutdown ROS node
        else:
            root.after(100, check_shutdown)  # Re-check the flag after 100ms

    # Call check_shutdown periodically
    root.after(100, check_shutdown)              

    # Create the main container
    main_container = tk.PanedWindow(root, orient='vertical')
    create_buttons_intro(intro_frame, intro_session_words, tts_client)
    create_single_random_button(one_frame, one_session_words, tts_client)
    create_single_random_button(two_frame, two_session_words, tts_client)
    create_single_random_button(three_frame, three_session_words, tts_client)
    create_buttons(four_frame, four_session_words, tts_client)
    create_buttons_multi_row(five_frame, five_session_words, tts_client)


    # Function to handle button press for text input
    def handle_text_input(event):
        typed_text = message_entry.get()  # Get the text from the entry widget
        print(typed_text)
        button_clicked(typed_text, tts_client)  # Use the same function to print/log the message
        entry_widget.delete(0, tk.END)

    entry_widget=message_entry
    # Bind the Enter key to the handle_text_input function
    message_entry.bind('<Return>', handle_text_input)

    # Start the GUI event loop
    root.mainloop()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)  # Register the signal handler for SIGINT

    rospy.init_node("gui")
    gui_main()
