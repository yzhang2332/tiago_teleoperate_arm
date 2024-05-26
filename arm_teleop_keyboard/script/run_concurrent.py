import multiprocessing
import subprocess

def run_gui_program():
    subprocess.run(["python3", "response_gui.py"])

def run_keyboard_control_program():
    subprocess.run(["python3", "kdl_ik.py"])

if __name__ == "__main__":
    gui_process = multiprocessing.Process(target=run_gui_program)
    keyboard_control_process = multiprocessing.Process(target=run_keyboard_control_program)

    gui_process.start()
    keyboard_control_process.start()

    gui_process.join()
    keyboard_control_process.join()