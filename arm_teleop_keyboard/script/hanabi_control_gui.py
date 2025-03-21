import tkinter as tk
import subprocess

class ButtonSelectorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Button Selector GUI")

        # Store selected values
        self.selected = {
            "box": None,
            "action": None,
            "row": None,
            "state": None
        }

        # Custom labels for action buttons
        self.action_labels = ["Forward", "Drop", "Play", "Discard", "Stack"]

        # Custom labels for action buttons
        self.State_labels = ["Normal", "Stacked", "Droped"]

        # Row 1 - Box Number Buttons (1 to 5)
        tk.Label(root, text="Select Box Number:", font=("Arial", 12)).grid(row=0, column=0, columnspan=5, sticky="w", padx=5)
        self.box_buttons = self.create_button_row("Box", 5, 1, self.select_box)

        # Row 2 - Buttons 1 and 2
        tk.Label(root, text="Pick Row:", font=("Arial", 12)).grid(row=2, column=0, columnspan=5, sticky="w", padx=5)
        self.third_row_buttons = self.create_button_row("row", 2, 3, self.select_third)

        # Row 3 - State Buttons 1 to 3
        tk.Label(root, text="Select State:", font=("Arial", 12)).grid(row=4, column=0, columnspan=5, sticky="w", padx=5)
        self.state_buttons = self.create_button_row("State", 3, 5, self.select_state, self.State_labels)

        # Row 4 - Action Buttons (custom labels)
        tk.Label(root, text="Choose Action:", font=("Arial", 12)).grid(row=6, column=0, columnspan=5, sticky="w", padx=5)
        self.action_buttons = self.create_button_row("Action", 5, 7, self.select_action, self.action_labels)

        # Row 5 - Go Button
        go_button = tk.Button(root, text="Go!", bg="orange", width=20, height=2, command=self.run_action)
        go_button.grid(row=8, column=0, columnspan=5, pady=10)

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

    def select_button(self, buttons, index, key):
        for btn in buttons:
            btn.config(bg="lightgray")
        buttons[index].config(bg="green")
        self.selected[key] = index + 1

    def select_box(self, index):
        self.select_button(self.box_buttons, index, "box")

    def select_action(self, index):
        self.select_button(self.action_buttons, index, "action")

    def select_third(self, index):
        self.select_button(self.third_row_buttons, index, "row")

    def select_state(self, index):
        self.select_button(self.state_buttons, index, "state")

    def run_action(self):
        if None in self.selected.values():
            print("Please select one button in each row before pressing Go!")
            return

        # Use the label instead of the index for the action
        action_label = self.action_labels[self.selected["action"] - 1]
        State_label = self.State_labels[self.selected["state"] - 1]
        args = [
            str(self.selected["box"]),
            action_label,
            str(self.selected["row"]),
            State_label
        ]

        print(f"Running hanabi_control.py with arguments: {args}")
        subprocess.run(["python", "hanabi_control.py"] + args)

# Run the GUI
if __name__ == "__main__":
    root = tk.Tk()
    app = ButtonSelectorApp(root)
    root.mainloop()
