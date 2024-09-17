import tkinter as tk

class CustomProgressBar(tk.Canvas):
    def __init__(self, parent, width, height, max_value, initial_value=0, **kwargs):
        super().__init__(parent, width=width, height=height, **kwargs)
        self.configure(bg='white', highlightthickness=0)

        self.max_value = max_value
        self.current_value = initial_value
        self.width = width
        self.height = height

        # Calculate positions
        self.x_start = 10
        self.x_end = width - 10
        self.y_center = height / 2
        self.bar_width = self.x_end - self.x_start
        self.zero_position = self.bar_width / 2

        # Draw outline
        self.create_rectangle(self.x_start, self.y_center - 10, self.x_end, self.y_center + 10, outline='black', width=2)

        # Draw initial bar
        self.update_bar()

    def update_bar(self):
        # Clear previous bar
        self.delete("bar")

        # Calculate length of filled bar
        bar_length = (abs(self.current_value) / self.max_value) * self.zero_position

        # Determine color based on value
        if self.current_value >= 0:
            bar_color = 'blue'
            bar_start = self.zero_position
        else:
            bar_color = 'red'
            bar_start = self.zero_position - bar_length

        # Draw filled bar
        self.create_rectangle(bar_start, self.y_center - 10, bar_start + bar_length, self.y_center + 10, fill=bar_color, outline='', tags="bar")

    def set_value(self, value):
        self.current_value = value
        self.update_bar()

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("Custom Progress Bar")
        self.root.geometry("400x200")

        self.progress_bar = CustomProgressBar(self.root, width=300, height=40, max_value=100, initial_value=0)
        self.progress_bar.pack(pady=20)

        self.value_label = tk.Label(self.root, text="Speed: 0", font=('Helvetica', 12))
        self.value_label.pack()

        self.update_button = tk.Button(self.root, text="Increase Speed", command=self.increase_speed)
        self.update_button.pack(pady=10)

        self.decrease_button = tk.Button(self.root, text="Decrease Speed", command=self.decrease_speed)
        self.decrease_button.pack()

    def increase_speed(self):
        if self.progress_bar.current_value < self.progress_bar.max_value:
            self.progress_bar.set_value(self.progress_bar.current_value + 10)
            self.update_value_label()

    def decrease_speed(self):
        if self.progress_bar.current_value > -self.progress_bar.max_value:
            self.progress_bar.set_value(self.progress_bar.current_value - 10)
            self.update_value_label()

    def update_value_label(self):
        self.value_label.config(text=f"Speed: {self.progress_bar.current_value}")

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
