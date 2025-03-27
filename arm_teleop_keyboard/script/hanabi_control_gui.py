import tkinter as tk
import subprocess

class ButtonSelectorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Button Selector GUI")
        self.process = None  # To store the subprocess

        # Store selected values
        self.selected = {
            "box": None,
            "action": None,
            "row": None,
            "state": None,
            "move_target": None  # Only used if action is "Move"
        }

        # Action labels including "Move" and "Stand"
        self.action_labels = ["Forward", "Drop", "Play", "Discard", "Stack", "Backward", "Unstack", "Move", "Stand"]
        self.State_labels = ["Normal", "Stacked", "Droped"]

        # Row 0 - Box Number Buttons
        tk.Label(root, text="Select Box Number:", font=("Arial", 12)).grid(row=0, column=0, columnspan=8, sticky="w", padx=5)
        self.box_buttons = self.create_button_row("Box", 5, 1, self.select_box)

        # Row 1 - Row Selector Buttons
        tk.Label(root, text="Pick Row:", font=("Arial", 12)).grid(row=2, column=0, columnspan=8, sticky="w", padx=5)
        self.third_row_buttons = self.create_button_row("row", 2, 3, self.select_third)

        # Row 2 - State Buttons
        tk.Label(root, text="Select State:", font=("Arial", 12)).grid(row=4, column=0, columnspan=8, sticky="w", padx=5)
        self.state_buttons = self.create_button_row("State", 3, 5, self.select_state, self.State_labels)

        # Row 3 - Action Buttons (including Stand)
        tk.Label(root, text="Choose Action:", font=("Arial", 12)).grid(row=6, column=0, columnspan=8, sticky="w", padx=5)
        self.action_buttons = self.create_button_row("Action", len(self.action_labels), 7, self.select_action, self.action_labels)

        # Row 4 - Move Target (only shown if "Move" is selected)
        self.move_target_label = tk.Label(root, text="Select Move Target:", font=("Arial", 12))
        self.move_target_buttons = self.create_button_row("Target", 5, 9, self.select_move_target)
        self.hide_move_target_buttons()

        # Row 5 - Go and Stop Buttons (fixed on row 10 to avoid layout clash)
        go_button = tk.Button(root, text="Go!", bg="orange", width=20, height=2, command=self.run_action)
        go_button.grid(row=10, column=0, columnspan=3, pady=10)

        stop_button = tk.Button(root, text="Stop", bg="red", fg="white", width=20, height=2, command=self.stop_action)
        stop_button.grid(row=10, column=5, columnspan=3, pady=10)

    def make_command(self, func, index):
        return lambda: func(index)

    def create_button_row(self, name, count, row, command_func, labels=None):
        buttons = []
        for i in range(count):
            label = labels[i] if labels else f"{name} {i+1}"
            btn = tk.Button(self.root, text=label, width=12, height=2,
                            command=self.make_command(command_func, i), bg="lightgray")
            btn.grid(row=row, column=i, padx=5, pady=5)
            buttons.append(btn)
        return buttons

    def hide_move_target_buttons(self):
        self.move_target_label.grid_forget()
        for btn in self.move_target_buttons:
            btn.grid_forget()

    def show_move_target_buttons(self):
        self.move_target_label.grid(row=8, column=0, columnspan=8, sticky="w", padx=5)
        for i, btn in enumerate(self.move_target_buttons):
            btn.grid(row=9, column=i, padx=5, pady=5)

    def select_button(self, buttons, index, key):
        for btn in buttons:
            btn.config(bg="lightgray")
        buttons[index].config(bg="green")
        self.selected[key] = index + 1

    def select_box(self, index):
        self.select_button(self.box_buttons, index, "box")

    def select_action(self, index):
        self.select_button(self.action_buttons, index, "action")
        action_label = self.action_labels[index]
        if action_label == "Move":
            self.show_move_target_buttons()
        else:
            self.hide_move_target_buttons()
            self.selected["move_target"] = None

    def select_third(self, index):
        self.select_button(self.third_row_buttons, index, "row")

    def select_state(self, index):
        self.select_button(self.state_buttons, index, "state")

    def select_move_target(self, index):
        self.select_button(self.move_target_buttons, index, "move_target")

    def run_action(self):
        if None in (self.selected["box"], self.selected["action"], self.selected["row"], self.selected["state"]):
            print("Please select one button in each row before pressing Go!")
            return

        action_label = self.action_labels[self.selected["action"] - 1]
        state_label = self.State_labels[self.selected["state"] - 1]

        args = [
            str(self.selected["box"]),
            action_label,
            str(self.selected["row"]),
            state_label
        ]

        if action_label == "Move":
            if self.selected["move_target"] is None:
                print("Please select a move target for the Move action.")
                return
            args.append(str(self.selected["move_target"]))

        print(f"Running hanabi_control.py with arguments: {args}")
        self.process = subprocess.Popen(["python", "hanabi_control.py"] + args)

    def stop_action(self):
        if self.process and self.process.poll() is None:
            print("Stopping the process...")
            self.process.terminate()
            self.process = None
        else:
            print("No process is currently running.")

# Run the GUI
if __name__ == "__main__":
    root = tk.Tk()
    app = ButtonSelectorApp(root)
    root.mainloop()
