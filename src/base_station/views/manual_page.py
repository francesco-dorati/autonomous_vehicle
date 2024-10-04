import tkinter as tk 
from PIL import Image, ImageTk

from .camera_frame import CameraFrame

class ManualPage(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.config(borderwidth=1, relief='solid')

        # UPPER FRAME
        self.upper_frame = tk.Frame(self, borderwidth=2, relief='solid')
        self.upper_frame.pack(side='top', fill='x', padx=20, pady=10)
        self.title = tk.Label(self.upper_frame, text="Manual Mode", font=("Arial", 26))
        self.title.pack(side='left', pady=20, padx=20)
        self.start_button = tk.Button(self.upper_frame, text="Start", font=("Arial", 18))
        self.stop_button = tk.Button(self.upper_frame, text="Stop", font=("Arial", 18))
        self.start_button.pack(side='right', pady=20, padx=20)

        # MAIN FRAME
        self.main_frame = tk.Frame(self)
        self.main_frame.pack(side='top', fill='both', expand=True, padx=10)
        padding = 0.01
        left_portion = 0.65
        # LEFT FRAME
        self.left_frame = tk.Frame(self.main_frame)
        self.left_frame.place(relx=padding, rely=padding, relwidth=(left_portion-2*padding), relheight=(1-2*padding))
        self.camera_frame = CameraFrame(self.left_frame)
        self.camera_frame.pack(side='top', fill='both', expand=True, pady=10)
        self.controls_frame = ControlsFrame(self.left_frame)
        self.controls_frame.pack(side='bottom', fill='x')

        # RIGHT FRAME
        self.right_frame = tk.Frame(self.main_frame)
        self.right_frame.place(relx=left_portion, rely=padding, relwidth=(1-left_portion-2*padding), relheight=(1-2*padding))
        self.odometry_frame = OdometryFrame(self.right_frame)
        self.odometry_frame.pack(side='top', fill='x', pady=10)
        self.sensors_frame = SensorsFrame(self.right_frame)
        self.sensors_frame.pack(side='bottom', fill='x', pady=10)
        
        # self.data_frame = DataFrame(self.side_frame)

        # self.side_frame = tk.Frame(self.main_frame, width=200)
        # self.data_frame = DataFrame(self.side_frame)
        # self.controls_frame = ControlsFrame(self.side_frame)

        # self.exit_button = tk.Button(self, text="Exit Manual Controller", font=("Arial", 16))
        
        # self.main_frame.pack(pady=10, padx=15, expand=True, fill='both')

        # self.camera_frame.pack(side='left', fill='both', expand=True, padx=10)
        # self.

        # self.side_frame.pack(side='left', fill='y', padx=7)
        # self.data_frame.pack(side='top', fill='both', expand=True)
        # self.controls_frame.pack(side='bottom', fill='x', pady=10)
        # # self.camera_frame.grid(row=0, column=0, sticky='nsew', padx=7)        
        # # self.data_frame.grid(row=0, column=1, sticky='nsew', padx=7)
        # # self.control_frame.grid_columnconfigure(0, weight=4)
        # # self.control_frame.grid_columnconfigure(1, weight=2)
        # # self.control_frame.grid_rowconfigure(0, weight=5)
        # self.exit_button.pack(side='bottom', pady=20)

    def start(self):  
        self.start_button.pack_forget()
        self.stop_button.pack(side='right', pady=20, padx=20)
        self.controls_frame.enable()
        self.odometry_frame.enable()
    
    def stop(self):
        self.stop_button.pack_forget()
        self.start_button.pack(side='right', pady=20, padx=20)
        self.controls_frame.disable()
        self.odometry_frame.disable()
    

    def show(self):
        self.pack(fill='both', expand=True)

    def hide(self):
        self.pack_forget()



        #         # Resize the CameraFrame based on the dimensions of the image


class ControlsFrame(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.config(borderwidth=2, relief='solid')
        self.pack()

        self.key_text_size = 15

        # wasd Frame
        self.wasd_frame = tk.Frame(self)
        self.wasd_frame.pack(side='left', fill='y', pady=30, padx=40)
        # W
        self.forward = tk.Label(self.wasd_frame, text="W ↑", font=("Arial", self.key_text_size))
        self.forward.grid(row=0, column=1, padx=8, pady=8)
        # S
        self.backward = tk.Label(self.wasd_frame, text="S ↓", font=("Arial", self.key_text_size))
        self.backward.grid(row=1, column=1, padx=8, pady=8)
        # A
        self.left = tk.Label(self.wasd_frame, text="A ←", font=("Arial", self.key_text_size))
        self.left.grid(row=1, column=0, padx=8, pady=8)
        # D
        self.right = tk.Label(self.wasd_frame, text="D →", font=("Arial", self.key_text_size))
        self.right.grid(row=1, column=2, padx=8, pady=8)

        # Boost Frame
        self.boost_frame = tk.Frame(self)
        self.boost_frame.pack(side='left', fill='y', pady=20, padx=20)
        self.align_frame = tk.Frame(self.boost_frame)
        self.align_frame.grid(row=0, column=0)
        self.shift = tk.Label(self.align_frame, text="shift - SLOW", font=("Arial", self.key_text_size),
                 bg='lightgray', relief='raised', width=15, anchor='w', justify='left')
        self.shift.grid(row=0, column=0, pady=10, sticky='w')
        self.boost = tk.Label(self.align_frame, text="space - FAST", font=("Arial", self.key_text_size),
                 bg='lightgray', relief='raised', width=30, anchor='w', justify='left')
        self.boost.grid(row=1, column=0, pady=10, sticky='w')

        self.disable()
    
    def enable(self):
        self.set('all', 0)

    def disable(self):
        self.set('all', -1)

    def set(self, key, show):
        if key == 'fwd' or key == 'all':
            if show == 1:
                self.forward.config(state='normal', bg='black', fg='white', relief='sunken')
            elif show == 0:
                self.forward.config(state='normal', bg='lightgray', fg='black', relief='raised')
            elif show == -1:
                self.forward.config(state='disabled', bg='lightgray', fg='black', relief='sunken')
        if key == 'bwd' or key == 'all':
            if show == 1:
                self.backward.config(state='normal', bg='black', fg='white', relief='sunken')
            elif show == 0:
                self.backward.config(state='normal', bg='lightgray', fg='black', relief='raised')
            elif show == -1:
                self.backward.config(state='disabled', bg='lightgray', fg='black', relief='sunken')
        if key == 'left' or key == 'all':
            if show == 1:
                self.left.config(state='normal', bg='black', fg='white', relief='sunken')
            elif show == 0:
                self.left.config(state='normal', bg='lightgray', fg='black', relief='raised')
            elif show == -1:
                self.left.config(state='disabled', bg='lightgray', fg='black', relief='sunken')
        if key == 'right' or key == 'all':
            if show == 1:
                self.right.config(state='normal', bg='black', fg='white', relief='sunken')
            elif show == 0:
                self.right.config(state='normal', bg='lightgray', fg='black', relief='raised')
            elif show == -1:
                self.right.config(state='disabled', bg='lightgray', fg='black', relief='sunken')
        if key == 'slow' or key == 'all':
            if show == 1:
                self.shift.config(state='normal', bg='black', fg='white', relief='sunken')
            elif show == 0:
                self.shift.config(state='normal', bg='lightgray', fg='black', relief='raised')
            elif show == -1:
                self.shift.config(state='disabled', bg='lightgray', fg='black', relief='sunken')
        if key == 'fast' or key == 'all':
            if show == 1:
                self.boost.config(state='normal', bg='black', fg='white', relief='sunken')
            elif show == 0:
                self.boost.config(state='normal', bg='lightgray', fg='black', relief='raised')
            elif show == -1:
                self.boost.config(state='disabled', bg='lightgray', fg='black', relief='sunken')
    

class OdometryFrame(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.config(borderwidth=1, relief='solid')

        # VELOCITY
        self.velocity_frame = tk.Frame(self, borderwidth=1, relief='solid')
        # self.velocity_frame.grid(row=0, column=0, sticky='nsew')
        self.velocity_frame.pack(side='left', fill='both', expand=True)
        self.velocity_label = tk.Label(self.velocity_frame, text="Velocity", font=("Arial", 16))
        self.velocity_label.pack(side='top', fill='x', pady=10, padx=35)
        self.velocity = tk.Label(self.velocity_frame, text="-.--", font=("Arial", 22))
        self.velocity.pack(side='top', fill='x', pady=10, padx=35)
        self.unit_label = tk.Label(self.velocity_frame, text="cm/s", font=("Arial", 16))
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
        self.x_unit = tk.Label(self.x_frame, text="  cm", font=("Arial", units_size))
        self.x_unit.pack(side='left')
        # Y
        self.y_frame = tk.Frame(self.coord_frame)
        self.y_frame.pack(side='top', fill='x', pady=5, padx=30)
        self.y_label = tk.Label(self.y_frame, text="y:  ", font=("Arial", units_size))
        self.y_label.pack(side='left')
        self.y = tk.Label(self.y_frame, text="--", font=("Arial", units_size))
        self.y.pack(side='left')
        self.y_unit = tk.Label(self.y_frame, text="  cm", font=("Arial", units_size))
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

    def enable(self):
        return
    
    def disable(self):
        self.velocity.config(text="--")
        self.x.config(text="--")
        self.y.config(text="--")
        self.theta.config(text="--")

    def update(self, vel, pos):
        self._update_velocity(vel)
        self._update_position(pos)
    
    def _update_velocity(self, vel):
        self.velocity.config(text=f"{int(vel)}")

    def _update_position(self, pos):
        self.x.config(text=f"{int(pos[0])}")
        self.y.config(text=f"{int(pos[1])}")
        self.theta.config(text=f"{int(pos[2])}")


class SensorsFrame(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.config(borderwidth=2, relief='solid')
    pass



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

# class ControlsFrame(tk.Frame):
#     def __init__(self, parent):
#         super().__init__(parent)
#         self.config(borderwidth=2, relief='solid', height=300)

#         self.text_size = 15
        
#         # CONTROLS FRAME
#         self.control_label = tk.Label(self, text="Controls: ", font=("Arial", 18))

#         self.keys_frame = tk.Frame(self)
#         self.fwd_label = tk.Label(self.keys_frame, text="W ↑", font=("Arial", self.text_size), bg='lightgray', relief='raised')
#         self.left_label = tk.Label(self.keys_frame, text="A ←", font=("Arial", self.text_size), bg='lightgray', relief='raised')
#         self.bwd_label = tk.Label(self.keys_frame, text="S ↓", font=("Arial", self.text_size), bg='lightgray', relief='raised')
#         self.right_label = tk.Label(self.keys_frame, text="D →", font=("Arial", self.text_size), bg='lightgray', relief='raised')
        
#         self.start_button = tk.Button(self, text="Start Control", font=("Arial", self.text_size))
#         self.stop_button = tk.Button(self, text="Stop Control", font=("Arial", self.text_size))


#         # PACK CONTROLS
#         self.control_label.pack(fill='x', side='top', pady=10)
#         self.keys_frame.pack(fill='x', side='top', pady=10, padx=30)
#         self.fwd_label.grid(row=0, column=1, padx=8, pady=8)
#         self.left_label.grid(row=1, column=0, padx=8, pady=8)
#         self.bwd_label.grid(row=1, column=1, padx=8, pady=8)
#         self.right_label.grid(row=1, column=2, padx=8, pady=8)
#         self.start_button.pack(fill='x', side='top', pady=10, padx=10)


#         # self.linear_label.pack(fill='x', side='left', pady=10)
#         # self.linear_velocity.pack(fill='x', side='left', pady=10)

#         # self.angular_velocity.pack(fill='x', side='right', pady=10)
#         # self.angular_label.pack(fill='x', side='right', pady=10)

#         # self.button = tk.Button(self, text="Start Data", font=("Arial", 16))
#         # self.button.pack(fill='none', pady=20, expand=True)


#     def forward(self, show):
#         if show:
#             self.fwd_label.config(bg='black', fg='white', relief='sunken')
#         else:
#             self.fwd_label.config(bg='lightgray', fg='black', relief='raised')

#     def backward(self, show):
#         if show:
#             self.bwd_label.config(bg='black', fg='white', relief='sunken')
#         else:
#             self.bwd_label.config(bg='lightgray', fg='black', relief='raised')

#     def left(self, show):
#         if show:
#             self.left_label.config(bg='black', fg='white', relief='sunken')
#         else:
#             self.left_label.config(bg='lightgray', fg='black', relief='raised')

#     def right(self, show):
#         if show:
#             self.right_label.config(bg='black', fg='white', relief='sunken')
#         else:
#             self.right_label.config(bg='lightgray', fg='black', relief='raised')
        
#     def disable(self):
#         self.start_button.pack(fill='x', side='top', pady=10, padx=10)
#         self.stop_button.pack_forget()
#         self.fwd_label.config(state='disabled')
#         self.bwd_label.config(state='disabled')
#         self.left_label.config(state='disabled')
#         self.right_label.config(state='disabled')

#     def enable(self):
#         self.stop_button.pack(fill='x', side='top', pady=10, padx=10)
#         self.start_button.pack_forget()
#         self.fwd_label.config(state='normal')
#         self.bwd_label.config(state='normal')
#         self.left_label.config(state='normal')
#         self.right_label.config(state='normal')


