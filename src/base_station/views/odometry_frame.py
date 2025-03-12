
import tkinter as tk
import math

class OdometryFrame(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.config(borderwidth=1, relief='raised')

        self.controller = controller

        # VELOCITY
        self.velocity_frame = tk.Frame(self, borderwidth=1, relief='solid')
        # self.velocity_frame.grid(row=0, column=0, sticky='nsew')
        self.velocity_frame.pack(side='left', fill='both', expand=True)
        self.velocity_label = tk.Label(self.velocity_frame, text="Velocity", font=("Arial", 16))
        self.velocity_label.pack(side='top', fill='x', pady=10, padx=35)
        self.velocity = tk.Label(self.velocity_frame, text="-.--", font=("Arial", 22))
        self.velocity.pack(side='top', fill='x', pady=10, padx=35)
        self.unit_label = tk.Label(self.velocity_frame, text="m/s", font=("Arial", 16))
        self.unit_label.pack(side='top', fill='x', pady=10, padx=35)

        # POSITION
        self.position_frame = tk.Frame(self, borderwidth=1, relief='solid')
        self.position_frame.pack(side='left', fill='both', expand=True)
        self.position_label = tk.Label(self.position_frame, text="Position", font=("Arial", 16))
        self.position_label.pack(side='top', fill='x', pady=10, padx=30)


        self.coord_frame = tk.Frame(self.position_frame)
        self.coord_frame.pack(side='top', fill='both', expand=True, pady=5, padx=10)
        units_size = 15
        # X
        self.x_frame = tk.Frame(self.coord_frame)
        self.x_frame.pack(side='top', fill='x', pady=5, padx=30)
        self.x_label = tk.Label(self.x_frame, text="x:  ", font=("Arial", units_size))
        self.x_label.pack(side='left')
        self.x = tk.Label(self.x_frame, text="--", font=("Arial", units_size))
        self.x.pack(side='left')
        self.x_unit = tk.Label(self.x_frame, text="  m", font=("Arial", units_size))
        self.x_unit.pack(side='left')
        # Y
        self.y_frame = tk.Frame(self.coord_frame)
        self.y_frame.pack(side='top', fill='x', pady=5, padx=30)
        self.y_label = tk.Label(self.y_frame, text="y:  ", font=("Arial", units_size))
        self.y_label.pack(side='left')
        self.y = tk.Label(self.y_frame, text="--", font=("Arial", units_size))
        self.y.pack(side='left')
        self.y_unit = tk.Label(self.y_frame, text="  m", font=("Arial", units_size))
        self.y_unit.pack(side='left')
        # THETA
        self.theta_frame = tk.Frame(self.coord_frame)
        self.theta_frame.pack(side='top', fill='x', pady=5, padx=30)
        self.theta_label = tk.Label(self.theta_frame, text="θ:  ", font=("Arial", units_size))
        self.theta_label.pack(side='left')
        self.theta = tk.Label(self.theta_frame, text="--", font=("Arial", units_size))
        self.theta.pack(side='left')
        self.theta_unit = tk.Label(self.theta_frame, text="  deg", font=("Arial", units_size))
        self.theta_unit.pack(side='left')

        self.reset_button = tk.Button(self.position_frame, text="Reset", font=("Arial", 16), command=self.controller.reset_odometry)
        self.reset_button.pack(side='top', fill='x', pady=10, padx=30)
        self.reset_button.config(state='disabled')


    def enable(self):
        self.reset_button.config(state='normal')
        
    
    def disable(self):
        self.reset_button.config(state='disabled')
        self.velocity.config(text="--")
        self.x.config(text="--")
        self.y.config(text="--")
        self.theta.config(text="--")

    def update(self, vel, state):
        x = state[0]
        y = state[1]
        theta = math.degrees(state[2])
        vel = state[3]
        w = state[4]

        self.x.config(text=f"{x:.3f}")
        self.y.config(text=f"{y:.3f}")
        self.theta.config(text=f"{int(theta)}")
        self.velocity.config(text=f"{vel:.3f}")


 
