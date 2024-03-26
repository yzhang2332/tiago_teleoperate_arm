import tkinter as tk
import rospy
import actionlib
from pal_interaction_msgs.msg import TtsAction, TtsGoal

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

def create_buttons_intro(master, words, tts_client):
    for word in words:
        button = tk.Button(master, text=word, command=lambda w=word: button_clicked(w, tts_client))
        button.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5, anchor='w')


def create_buttons(master, words, tts_client):
    """Create buttons in the specified master widget based on the words provided."""
    for word in words:
        button = tk.Button(master, text=word, command=lambda w=word: button_clicked(w, tts_client))
        button.pack(side=tk.LEFT, padx=5, pady=5)

def gui_main():

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


    main_container.add(intro_frame)
    main_container.add(one_frame)
    main_container.add(two_frame)
    main_container.add(three_frame)
    main_container.add(four_frame)
    main_container.add(five_frame)

    # Add a text description (label) to each frame
    intro_label = tk.Label(intro_frame, text="Introduction", bg='darkgray')
    one_label = tk.Label(one_frame, text="Direct/ISA Group", bg='lightgray')
    two_label = tk.Label(two_frame, text="Can you?", bg='darkgray')
    three_label = tk.Label(three_frame, text="Provide information", bg='lightgray')
    four_label = tk.Label(four_frame, text="Missed", bg='darkgray')
    five_label = tk.Label(five_frame, text="Help and others", bg='lightgray')

    # Pack the labels to the top of each frame
    intro_label.pack(side=tk.TOP, anchor='nw') 
    one_label.pack(side=tk.TOP, anchor='nw') 
    two_label.pack(side=tk.TOP, anchor='nw')
    three_label.pack(side=tk.TOP, anchor='nw')
    four_label.pack(side=tk.TOP, anchor='nw')
    five_label.pack(side=tk.TOP, anchor='nw')

    # Define the words for the buttons in both sessions
    intro_session_words = ["Hey, Mate! How are you? I’m so exciting to work with you. \nSo, what are we going to do?", "Alright, how can I help you first?", "We did a great job. Now what else are we going to do?", "I’m sorry, that’s too much detail to remember. \nLet’s get hands on the work, then you can fill me in when it’s need."]
    one_session_words = ["Sure.", "Yes, I can.", "I will work on that."]
    two_session_words = ["Yes, I'm able to do that.", "Yes, I will do it if you want."]
    three_session_words = ["I'm happy to hear that.", "It's good to know."]
    four_session_words = ["I'm sorry I don't understand.", "Sorry I missed that. Can you say it again?"]
    five_session_words = ["I need help.", "I can't reach that.", "What should I do next?", "Thank you."]

    # Create buttons in both sessions
    create_buttons_intro(intro_frame, intro_session_words, tts_client)
    create_buttons(one_frame, one_session_words, tts_client)
    create_buttons(two_frame, two_session_words, tts_client)
    create_buttons(three_frame, three_session_words, tts_client)
    create_buttons(four_frame, four_session_words, tts_client)
    create_buttons(five_frame, five_session_words, tts_client)

    # Start the GUI event loop
    root.mainloop()

if __name__ == "__main__":

    rospy.init_node("gui")
    gui_main()

#     # Define the words for your buttons
#     button_words = ["Hello", "World", "Click Me", "Press"]

#     # Create and pack the buttons into the window
#     for word in button_words:
#         button = create_button(root, word, tts_client)
#         button.pack()

#     # Start the GUI event loop
#     root.mainloop()

# if __name__ == "__main__":
#     main()
