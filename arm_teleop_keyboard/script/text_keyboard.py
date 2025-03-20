from pynput import keyboard
import time

# Initialize variables to track key press times
last_time = None

def on_press(key):
    global last_time
    
    # Get the current time
    current_time = time.time()
    
    # Calculate time difference and frequency
    if last_time is not None:
        time_diff = current_time - last_time
        frequency = 1 / time_diff if time_diff > 0 else 0
        print(f"Key pressed: {key}, Frequency: {frequency:.2f} Hz")
    else:
        print(f"Key pressed: {key}, Frequency: N/A (first press)")
    
    # Update the last press time
    last_time = current_time

with keyboard.Listener(on_press=on_press) as listener:
    listener.join()
