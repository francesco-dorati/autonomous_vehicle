import tkinter as tk       
class ControlsFrame(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.config(borderwidth=2, relief='raised')

        self.controller = controller
        # CHOICE FRAME
        self.choice_frame = tk.Frame(self, padx=150)
        self.choice_label = tk.Label(self.choice_frame, text="Choose Control Mode", font=("Arial", 15, "bold"))
        self.manual_button = tk.Button(self.choice_frame, text="Manual", font=("Arial", 15), command=lambda: self.controller.set_control("manual"))
        self.auto_button = tk.Button(self.choice_frame, text="Autonomous", font=("Arial", 15), command=lambda: self.controller.set_control("auto"))
        self.choice_label.pack(side='top', fill='x', pady=10)
        self.manual_button.pack(side='top', fill='x', padx=10)
        self.auto_button.pack(side='top', fill='x', padx=10)

        self.manual_frame = ManualFrame(self, controller)

        self.auto_frame = tk.Frame(self)
        self.auto_label = tk.Label(self.auto_frame, text="Autonomous Control Mode", font=("Arial", 15, "bold"))
        self.auto_label.pack(side='top', fill='x', pady=10)

    def show(self):
        self.pack(side='top', fill='x')
        self.show_choice()
        self.disable()
    
    def enable(self):
        self.choice_label.config(state='normal')
        self.manual_button.config(state='normal')
        self.auto_button.config(state='normal')
    def disable(self):
        self.show_choice()
        self.choice_label.config(state='disabled')
        self.manual_button.config(state='disabled')
        self.auto_button.config(state='disabled')
    
    def show_choice(self):
        self.choice_frame.pack(side='top', fill='x', pady=10)
        self.manual_frame.hide()
        self.auto_frame.pack_forget()
        
    def show_manual(self):
        self.choice_frame.pack_forget()
        self.manual_frame.show()
        self.auto_frame.pack_forget()
        
    def show_auto(self):
        self.choice_frame.pack_forget()
        self.manual_frame.hide()
        self.auto_frame.pack(side='top', fill='x', pady=10)
        
    def enable_auto(self):
        self.auto_button.config(state='normal')

    def disable_auto(self):
        self.auto_button.config(state='disabled')

class ManualFrame(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.title = tk.Label(self, text="Manual Control Mode", font=("Arial", 15, "bold"))
        self.key_text_size = 15

        self.frame = tk.Frame(self)
        self.controller = controller
        
        # self.keyboard_frame = tk.Frame(self)
        # wasd Frame
        self.keyboard_frame = tk.Frame(self.frame)
        # W
        self.forward = tk.Label(self.keyboard_frame, text="W ↑", font=("Arial", self.key_text_size))
        self.forward.grid(row=0, column=1, padx=8, pady=8)
        # S
        self.backward = tk.Label(self.keyboard_frame, text="S ↓", font=("Arial", self.key_text_size))
        self.backward.grid(row=1, column=1, padx=8, pady=8)
        # A
        self.left = tk.Label(self.keyboard_frame, text="A ←", font=("Arial", self.key_text_size))
        self.left.grid(row=1, column=0, padx=8, pady=8)
        # D
        self.right = tk.Label(self.keyboard_frame, text="D →", font=("Arial", self.key_text_size))
        self.right.grid(row=1, column=2, padx=8, pady=8)

        # self.joypad_label = tk.Label(self.joypad_frame, text="Joypad", font=("Arial", 15, "bold"))
        # self.joypad_label.pack(side='top', fill='x', pady=10)
        self.joypad_frame = tk.Frame(self.frame)
        self.joypad_canvas_size = 100
        self.joypad_point_radius = 10
        mid = self.joypad_canvas_size//2
        self.joypad_scale = mid - self.joypad_point_radius - 5
        self.joypad_canvas = tk.Canvas(self.joypad_frame, width=self.joypad_canvas_size, height=self.joypad_canvas_size, bg="lightgray")
        self.joypad_canvas.create_line(mid, 0, mid, self.joypad_canvas_size, fill="black")  # Y-axis
        self.joypad_canvas.create_line(0, mid, self.joypad_canvas_size, mid, fill="black")  # X-axis
        self.joypad_canvas.create_oval(mid - self.joypad_scale, mid - self.joypad_scale, mid + self.joypad_scale, mid + self.joypad_scale, outline="black", width=2)
        self.joypad_point = self.joypad_canvas.create_oval(0, 0, 0, 0, fill="gray")
        self.joypad_canvas.pack(side='left', fill='none')

        self.buttons_frame = tk.Frame(self.frame)
        self.switch_button = tk.Button(self.buttons_frame, font=("Arial", 16), width=15, height=1)
        self.switch_button.pack(side='top', fill='x', padx=10, pady=5)
        self.exit_button = tk.Button(self.buttons_frame, text="⛔  stop", font=("Arial", 16, "bold"), command=lambda: self.controller.set_control("off"),  width=15, height=1)
        self.exit_button.pack(side='top', fill='x', padx=10, pady=5)
    
    def show(self):
        self.pack(side='top', fill='x', pady=10)
        self.title.pack(side='top', fill='x', pady=10)
        self.frame.pack(side='top', fill='x')
        self.frame.grid_columnconfigure(0, weight=1)  # Keyboard area
        self.frame.grid_columnconfigure(1, weight=1)  # Empty space
        self.frame.grid_columnconfigure(2, weight=1)  # Buttons
        if self.controller.input_handler:
            self.controller.input_handler.set_input("keyboard")
        self.buttons_frame.grid(row=0, column=2, padx=20, pady=10, sticky='e')
    
    def hide(self):
        self.pack_forget()

    def show_keyboard(self):
        # self.keyboard_frame.pack(side='left', fill='y', pady=10, padx=20)
        self.keyboard_frame.grid(row=0, column=0, padx=20, pady=10, sticky='w')
        self.forward.config(state='normal', bg='lightgray', fg='black', relief='raised')
        # self.forward.grid(row=0, column=1, padx=8, pady=8)
        self.backward.config(state='normal', bg='lightgray', fg='black', relief='raised')
        # self.backward.grid(row=1, column=1, padx=8, pady=8)
        self.left.config(state='normal', bg='lightgray', fg='black', relief='raised')
        # self.left.grid(row=1, column=0, padx=8, pady=8)
        self.right.config(state='normal', bg='lightgray', fg='black', relief='raised')
        # self.right.grid(row=1, column=2, padx=8, pady=8)
        
        self.switch_button.config(text="🎮  joypad", command=lambda: self.controller.input_handler.set_input('joypad'))
    
    def hide_keyboard(self):
        self.keyboard_frame.grid_forget()

    def show_joypad(self):
        self.joypad_frame.grid(row=0, column=0, padx=20, pady=10, sticky='w')
        self.switch_button.config(text="⌨️  keyboard", command=lambda: self.controller.input_handler.set_input("keyboard"))
        # self.joypad_canvas.pack(side='left', fill='none', pady=10)

    def hide_joypad(self):
        self.joypad_frame.grid_forget()
    
    def set_joypad(self, j):
        x = j['x']
        y = j['y']
        mid = self.joypad_canvas_size//2
        canvas_x = mid - y * self.joypad_scale
        canvas_y = mid - x * self.joypad_scale 
        self.joypad_canvas.coords(self.joypad_point, canvas_x - self.joypad_point_radius, canvas_y - self.joypad_point_radius,
                           canvas_x + self.joypad_point_radius, canvas_y + self.joypad_point_radius)
        # self.joypad_label.config(text=f"Joypad: {x:.2f}, {y:.2f}")
    
    def connect_joypad(self):
        self.joypad_canvas.itemconfig(self.joypad_point, fill="red")

    def disconnect_joypad(self):
        self.joypad_canvas.itemconfig(self.joypad_point, fill="grey")    
    def set_key(self, k):
        x = k['x']
        y = k['y']
        if x == 1:
            self.forward.config(bg='black', fg='white', relief='sunken')
            self.backward.config(bg='lightgray', fg='black', relief='raised')
        elif x == 0:
            self.forward.config(bg='lightgray', fg='black', relief='raised')
            self.backward.config(bg='lightgray', fg='black', relief='raised')
        elif x == -1:
            self.forward.config(bg='lightgray', fg='black', relief='raised')
            self.backward.config(bg='black', fg='white', relief='sunken')
        
        if y == 1:
            self.left.config(bg='black', fg='white', relief='sunken')
            self.right.config(bg='lightgray', fg='black', relief='raised')
        elif y == 0:
            self.left.config(bg='lightgray', fg='black', relief='raised')
            self.right.config(bg='lightgray', fg='black', relief='raised')
        elif y == -1:
            self.left.config(bg='lightgray', fg='black', relief='raised')
            self.right.config(bg='black', fg='white', relief='sunken')

