import tkinter as tk
from .components.camera_frame import CameraFrame
from .controls_frame import ControlsFrame
from .mapping_frame import MappingFrame
from .display_frame import DisplayFrame

class MainPage(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.config(borderwidth=1, relief='solid')

        self.controller = controller

        # MAIN FRAME
        self.main_frame = tk.Frame(self)
        self.main_frame.pack(side='top', fill='both', expand=True, padx=10)
        padding = 0.01
        left_portion = 0.65

        ## LEFT FRAME
        self.left_frame = tk.Frame(self.main_frame)
        self.left_frame.place(relx=padding, rely=padding, relwidth=(left_portion-2*padding), relheight=(1-2*padding))

        # DISPLAY
        self.display_frame = DisplayFrame(self.left_frame, controller)
        self.display_frame.show()

        # CONTROLS
        self.controls_frame = ControlsFrame(self.left_frame, controller)
        self.controls_frame.show()

        ## RIGHT FRAME
        self.right_frame = tk.Frame(self.main_frame)
        self.right_frame.place(relx=left_portion, rely=padding, relwidth=(1-left_portion-2*padding), relheight=(1-2*padding))
        # CAMERA
        self.odometry_frame = OdometryFrame(self.right_frame)
        self.odometry_frame.pack(side='top', fill='x', pady=10)
        # MAPPING
        self.mapping_frame = MappingFrame(self.right_frame, controller)
        self.mapping_frame.show()
        
    def show(self):
        self.pack(fill='both', expand=True)
        self.mapping_frame.show()
        self.controls_frame.show()

    def connect(self):  
        # self.start_button.pack_forget()
        # self.stop_button.pack(side='right', pady=20, padx=20)
        self.mapping_frame.enable()
        self.controls_frame.enable()
        self.controls_frame.disable_auto()
    
    def disconnect(self):
        # self.stop_button.pack_forget()
        # self.start_button.pack(side='right', pady=20, padx=20)
        self.controls_frame.disable()
        self.mapping_frame.disable()
    
    def disable(self):
        self.controls_frame.disable()
        self.mapping_frame.disable()

    def set_map(self, map_name):
        if map_name is None:
            return
        self.mapping_frame.set_map(map_name)
        self.controls_frame.enable_auto()

    def discard_map(self):
        self.mapping_frame.discard_map()
        self.controls_frame.disable_auto()
   
    def set_control(self, type: str):
        if type == 'off':
            self.controls_frame.show_choice()
        elif type == 'manual':
            self.controls_frame.show_manual()
        elif type == 'auto':
            self.controls_frame.show_auto()





class ControlssFrame(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.config(borderwidth=2, relief='raised')
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
        self.config(borderwidth=1, relief='raised')

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
        self.config(borderwidth=2, relief='raised')

        self.map_label = tk.Label(self, text="map: ")
        self.title.pack(fill='x', pady=5)
        # Create a Canvas in the frame
        self.canvas = tk.Canvas(self)
        self.canvas.pack(fill='x', expand=True, pady=10)

        # Now you can create a rectangle on the Canvas
        x_pad = 30
        y_pad = 30
        car_width = 110
        car_height = 140
        sensors_width = 40
        sensors_height = 25
        car_x0 = x_pad
        car_y0 = y_pad
        car_x1 = x_pad+car_width
        car_y1 = y_pad+car_height
        sensors_positions = { # (x0, y0, x1, y1)
            'fl': (car_x0, car_y0, car_x0+sensors_width, car_y0+sensors_height),
            'fr': (car_x1-sensors_width, car_y0, car_x1, car_y0+sensors_height),
            'bl': (car_x0, car_y1-sensors_height, car_x0+sensors_width, car_y1),
            'br': (car_x1-sensors_width, car_y1-sensors_height, car_x1, car_y1)
         } 
        self.car = self.canvas.create_rectangle(car_x0, car_y0, car_x1, car_y1, fill='lightgray')
        self.sensors = [
            self.canvas.create_rectangle(*sensors_positions['fl'], fill='gray'),
            self.canvas.create_rectangle(*sensors_positions['fr'], fill='gray'),
            self.canvas.create_rectangle(*sensors_positions['bl'], fill='gray'),
            self.canvas.create_rectangle(*sensors_positions['br'], fill='gray')]
        
    def update(self, distances_cm):
        DARKRED = '#8B0000'
        DARKORANGE = '#FF8C00'

        print('Updating sensors')
        thresholds = [10, 15, 20, 30, 45, 1000]
        colors = [DARKRED, 'red', DARKORANGE, 'orange', 'yellow', 'green']
        
        for i, dist in enumerate(distances_cm):
            print("Distance", i, ":", dist)
            color = None
            for t in range(len(thresholds)-1):
                if thresholds[t] <= dist < thresholds[t+1]:
                    color = colors[t]
                    break

            print(f"Distance {i}:", dist, "Color:", color)
            self.canvas.itemconfig(self.sensors[i], fill=color)
    def disable(self):
        for sensor in self.sensors:
            self.canvas.itemconfig(sensor, fill='gray')

     # self.canvas.create_text(50, 50, text=str(dist), font=("Arial", 12), fill='black')


    #     sensor_positions = [
    #     (120, 170),  # Top-left
    #     (280, 170),  # Top-right
    #     (120, 330),  # Bottom-left
    #     (280, 330)   # Bottom-right
    # ]

    # # Create sensor squares
    # sensor_size = 40  # Size of the sensor squares

    # for index, (x, y) in enumerate(sensor_positions):
    #     # Choose color based on distance
    #     distance = distances[index]
    #     if distance < 50:
    #         color = 'gray'
    #         text_color = 'white'
    #     elif distance < 100:
    #         color = 'gray'
    #         text_color = 'black'
    #     elif distance < 150:
    #         color = 'gray'
    #         text_color = 'black'
    #     else:
    #         color = 'gray'
    #         text_color = 'black'

    #     # Draw the sensor square, ensuring they stay within the rectangle
    #     canvas.create_rectangle(x - sensor_size // 2, y - sensor_size // 2,
    #                             x + sensor_size // 2, y + sensor_size // 2,
    #                             fill=color)

    #     # Draw the distance text inside the square
    #     canvas.create_text(x, y, text=str(distance), font=("Arial", 12), fill=text_color)





class DataFrame(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.config(borderwidth=2, relief='raised')

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
