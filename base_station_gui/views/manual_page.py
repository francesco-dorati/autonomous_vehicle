import tkinter as tk 
from PIL import Image, ImageTk

class ManualPage(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.config(borderwidth=1, relief='solid')

        self.label = tk.Label(self, text="Manual Control", font=("Arial", 24))

        self.main_frame = tk.Frame(self, borderwidth=3)
        self.camera_frame = CameraFrame(self.main_frame)

        self.side_frame = tk.Frame(self.main_frame, width=200)
        self.data_frame = DataFrame(self.side_frame)
        self.controls_frame = ControlsFrame(self.side_frame)

        self.exit_button = tk.Button(self, text="Exit Manual Controller", font=("Arial", 16))
        

        self.label.pack(pady=15)    
        self.main_frame.pack(pady=10, padx=15, expand=True, fill='both')
        self.camera_frame.pack(side='left', fill='both', expand=True, padx=7)

        self.side_frame.pack(side='left', fill='y', padx=7)
        self.data_frame.pack(side='top', fill='both', expand=True)
        self.controls_frame.pack(side='bottom', fill='x', pady=10)
        # self.camera_frame.grid(row=0, column=0, sticky='nsew', padx=7)        
        # self.data_frame.grid(row=0, column=1, sticky='nsew', padx=7)
        # self.control_frame.grid_columnconfigure(0, weight=4)
        # self.control_frame.grid_columnconfigure(1, weight=2)
        # self.control_frame.grid_rowconfigure(0, weight=5)
        self.exit_button.pack(side='bottom', pady=20)

    

    def show(self):
        self.pack(fill='both', expand=True)

    def hide(self):
        self.pack_forget()


class CameraFrame(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.config(borderwidth=2, bg='lightgray', relief='solid')

        self.start_button = tk.Button(self, text="Start Camera", font=("Arial", 16))
        self.start_button.pack(fill='none', pady=20, expand=True)

        self.stop_button = tk.Button(self, text="Stop Camera", font=("Arial", 16))

        self.image_label = tk.Label(self)
    
    def start(self):
        self.start_button.pack_forget()
        self.stop_button.pack(fill='none', pady=20, expand=True)
        self.image_label.pack()

    def disable(self):
        self.stop_button.pack_forget()
        self.start_button.pack(fill='none', pady=20, expand=True)

    def update_image(self, image):
        imgtk = ImageTk.PhotoImage(image=image)
        self.image_label.imgtk = imgtk
        self.image_label.configure(image=imgtk)


class DataFrame(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.config(borderwidth=2, relief='solid')

        self.text_size = 15

        # VELOCITY FRAME
        self.velocity_frame = tk.Frame(self)
        self.velocity_label = tk.Label(self.velocity_frame, text="Velocity: ", font=("Arial", self.text_size+2))
        
        self.linear_frame = tk.Frame(self.velocity_frame)
        self.linear_label = tk.Label(self.linear_frame, text="Linear: ", font=("Arial", self.text_size))
        self.linear_velocity = tk.Label(self.linear_frame, text="0.00 [cm/s]", font=("Arial", self.text_size))
        
        self.angular_frame = tk.Frame(self.velocity_frame)
        self.angular_label = tk.Label(self.angular_frame, text="Angular: ", font=("Arial", self.text_size))
        
        # PACK VELOCITY
        self.angular_velocity = tk.Label(self.angular_frame, text="0.00 [deg/s]", font=("Arial", self.text_size))
        self.velocity_frame.pack(fill='x', side='top', pady=10)
        self.velocity_label.pack(fill='x', side='top', pady=10)
        self.linear_frame.pack(fill='x', side='top', pady=10, padx=30)
        self.linear_label.pack(fill='x', side='left', padx=5)
        self.linear_velocity.pack(fill='x', side='left', padx=5)
        self.angular_frame.pack(fill='x', side='top', pady=10, padx=30)
        self.angular_label.pack(fill='x', side='left', padx=5)
        self.angular_velocity.pack(fill='x', side='left', padx=5)

        # POSITION FRAME
        self.position_frame = tk.Frame(self)
        self.position_label = tk.Label(self.position_frame, text="Position: ", font=("Arial", self.text_size+2))
        
        self.x_frame = tk.Frame(self.position_frame)
        self.x_label = tk.Label(self.x_frame, text="X: ", font=("Arial", self.text_size))
        self.pos_x = tk.Label(self.x_frame, text="0.00 [m]", font=("Arial", self.text_size))
        
        self.y_frame = tk.Frame(self.position_frame)
        self.y_label = tk.Label(self.y_frame, text="Y: ", font=("Arial", self.text_size))
        self.pos_y = tk.Label(self.y_frame, text="0.00 [m]", font=("Arial", self.text_size))

        self.theta_frame = tk.Frame(self.position_frame)
        self.theta_label = tk.Label(self.theta_frame, text="θ: ", font=("Arial", self.text_size))
        self.pos_theta = tk.Label(self.theta_frame, text="0.00 [deg]", font=("Arial", self.text_size))

        # PACK POSITION
        self.position_frame.pack(fill='x', side='top', pady=10)
        self.position_label.pack(fill='x', side='top', pady=10)
        self.x_frame.pack(fill='x', side='top', pady=10, padx=30)
        self.x_label.pack(fill='x', side='left', padx=5)
        self.pos_x.pack(fill='x', side='left', padx=5)
        self.y_frame.pack(fill='x', side='top', pady=10, padx=30)
        self.y_label.pack(fill='x', side='left', padx=5)
        self.pos_y.pack(fill='x', side='left', padx=5)
        self.theta_frame.pack(fill='x', side='top', pady=10, padx=30)
        self.theta_label.pack(fill='x', side='left', padx=5)
        self.pos_theta.pack(fill='x', side='left', padx=5)

    def update(self, vel, pos):
        self._update_velocity(vel[0], vel[1])
        self._update_position(pos[0], pos[1], pos[2])

    def _update_velocity(self, linear, angular):
        self.linear_velocity.config(text=f"{linear:.2f} [cm/s]")
        self.angular_velocity.config(text=f"{angular:.2f} [deg/s]")

    def _update_position(self, x, y, theta):
        self.pos_x.config(text=f"{x:.2f} [m]")
        self.pos_y.config(text=f"{y:.2f} [m]")
        self.pos_theta.config(text=f"{theta:.2f} [deg]")
    
    def disable(self):
        self.linear_velocity.config(text=f"-.-- [cm/s]")
        self.angular_velocity.config(text=f"-.-- [deg/s]")
        self.pos_x.config(text=f"-.-- [m]")
        self.pos_y.config(text=f"-.-- [m]")
        self.pos_theta.config(text=f"-.-- [deg]")

class ControlsFrame(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.config(borderwidth=2, relief='solid', height=300)

        self.text_size = 15
        
        # CONTROLS FRAME
        self.control_label = tk.Label(self, text="Controls: ", font=("Arial", 18))

        self.keys_frame = tk.Frame(self)
        self.fwd_label = tk.Label(self.keys_frame, text="W ↑", font=("Arial", self.text_size), bg='lightgray', relief='raised')
        self.left_label = tk.Label(self.keys_frame, text="A ←", font=("Arial", self.text_size), bg='lightgray', relief='raised')
        self.bwd_label = tk.Label(self.keys_frame, text="S ↓", font=("Arial", self.text_size), bg='lightgray', relief='raised')
        self.right_label = tk.Label(self.keys_frame, text="D →", font=("Arial", self.text_size), bg='lightgray', relief='raised')
        
        self.start_button = tk.Button(self, text="Start Control", font=("Arial", self.text_size))
        self.stop_button = tk.Button(self, text="Stop Control", font=("Arial", self.text_size))


        # PACK CONTROLS
        self.control_label.pack(fill='x', side='top', pady=10)
        self.keys_frame.pack(fill='x', side='top', pady=10, padx=30)
        self.fwd_label.grid(row=0, column=1, padx=8, pady=8)
        self.left_label.grid(row=1, column=0, padx=8, pady=8)
        self.bwd_label.grid(row=1, column=1, padx=8, pady=8)
        self.right_label.grid(row=1, column=2, padx=8, pady=8)
        self.stop_button.pack(fill='x', side='top', pady=10, padx=10)


        # self.linear_label.pack(fill='x', side='left', pady=10)
        # self.linear_velocity.pack(fill='x', side='left', pady=10)

        # self.angular_velocity.pack(fill='x', side='right', pady=10)
        # self.angular_label.pack(fill='x', side='right', pady=10)

        # self.button = tk.Button(self, text="Start Data", font=("Arial", 16))
        # self.button.pack(fill='none', pady=20, expand=True)


    def forward(self, show):
        if show:
            self.fwd_label.config(bg='black', fg='white', relief='sunken')
        else:
            self.fwd_label.config(bg='lightgray', fg='black', relief='raised')

    def backward(self, show):
        if show:
            self.bwd_label.config(bg='black', fg='white', relief='sunken')
        else:
            self.bwd_label.config(bg='lightgray', fg='black', relief='raised')

    def left(self, show):
        if show:
            self.left_label.config(bg='black', fg='white', relief='sunken')
        else:
            self.left_label.config(bg='lightgray', fg='black', relief='raised')

    def right(self, show):
        if show:
            self.right_label.config(bg='black', fg='white', relief='sunken')
        else:
            self.right_label.config(bg='lightgray', fg='black', relief='raised')
        
    def disable(self):
        self.start_button.pack(fill='x', side='top', pady=10, padx=10)
        self.stop_button.pack_forget()
        self.fwd_label.config(state='disabled')
        self.bwd_label.config(state='disabled')
        self.left_label.config(state='disabled')
        self.right_label.config(state='disabled')

    def enable(self):
        self.stop_button.pack(fill='x', side='top', pady=10, padx=10)
        self.start_button.pack_forget()
        self.fwd_label.config(state='normal')
        self.bwd_label.config(state='normal')
        self.left_label.config(state='normal')
        self.right_label.config(state='normal')


