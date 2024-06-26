import tkinter as tk
from tkinter import ttk
import socket
import time

HOSTNAME = "172.20.10.7"
MAIN_PORT = 5500

class RobotGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Robot Control Panel")
        self.geometry("900x600")

        style = ttk.Style()
        style.theme_use('aqua')
        # style.configure("Exit.TButton", background="red", foreground="white")

        self.main_socket = None

        self.init_widgets()

        self.check_connection()


    def init_widgets(self):
        style = ttk.Style()
        style.theme_use('aqua')


        # SIDEBAR
        self.sidebar = tk.Frame(self, width=200, bg='gray', borderwidth=2)
        self.sidebar.pack(fill='both', side='left', padx=5, pady=5)

        # self.status_label = tk.Label(self.sidebar, text=f"Status: Not Connected", bg='gray')
        # self.status_label.pack(side='top', fill='x', padx=5, pady=10)

        # self.connect_button = tk.Button(self.sidebar, text="Connect", command=self.connect)
        # self.connect_button.pack(side='top', fill='x', padx=10, pady=10)

        self.battery_label = tk.Label(self.sidebar, text=f"Battery: 100%", bg='gray')
        self.battery_label.pack(side='top', fill='x', padx=10, pady=10)

        self.exit_button = ttk.Button(self.sidebar, text="Exit", command=lambda: exit(0))
        self.exit_button.pack(side='bottom', fill='x', padx=20, pady=5)


        # MAIN PAGE
        self.main_page = tk.Frame(self, bg='white', borderwidth=2)
        self.main_page.pack(side='top', fill='both', expand=True, padx=5, pady=5)

        # HEADER
        self.header_frame = tk.Frame(self.main_page, bg='white', borderwidth=2)
        self.header_frame.pack(side='top', fill='x', expand=True, padx=5, pady=5, anchor='nw')
        # CONNECTION STATUS
        self.connection_frame = tk.Frame(self.header_frame, bg='white', borderwidth=2, relief='raised')
        self.connection_frame.pack(side='left', fill="x", expand=True, padx=5, pady=5)
        self.status_text = tk.Label(self.connection_frame, text=f"Status: ", bg='white', fg="black")
        self.status_text.pack(side='left', fill='y', padx=5, pady=10)
        self.status_label = tk.Label(self.connection_frame, text=f"Not Connected", bg='white', fg="black")
        self.status_label.pack(side='right', fill='y', padx=5, pady=10)
        # POWER BUTTONS
        self.settings_button = ttk.Button(self.header_frame, text="Settings")
        self.settings_button.pack(side='right', fill='y', padx=5, pady=10)
        self.connect_button = ttk.Button(self.header_frame, text="Connect", command=self.connect)
        self.connect_button.pack(side='right', fill='y', padx=5, pady=10)

        # MAIN FRAME
        self.not_connected_frame = tk.Frame(self.main_page, bg='white', borderwidth=2)
        self.not_connected_frame.pack(side='top', fill='both', expand=True, padx=5, pady=5)

        self.main_frame = tk.Frame(self.main_page, bg='white', borderwidth=2)
        self.manual_button = ttk.Button(self.main_frame, text="Start Manual Control", command=self.start_manual)
        self.manual_button.pack(side='top', fill='x', padx=5, pady=5)

        self.manual_frame = tk.Frame(self.main_page, bg='white', borderwidth=2)

        self.data_frame = tk.Frame(self.manual_frame, bg='white', borderwidth=2, relief='raised')
        self.data_frame.pack(side='right', fill='y', expand=True, padx=5, pady=5, anchor='ne')
        # VELOCITY
        self.velocity_frame = tk.Frame(self.data_frame, bg='white', borderwidth=2)
        self.velocity_frame.pack(side='top', fill='x', expand=True, padx=5, pady=5)
        self.velocity_text = tk.Label(self.velocity_frame, text="Velocities: ", bg='white', fg="black")
        self.velocity_text.pack(side='top', fill='x', padx=5, pady=5)
        self.lin_velocity_label = tk.Label(self.velocity_frame, text="0 [cm/s]", bg='white', fg="black")
        self.lin_velocity_label.pack(side='left', fill='x', padx=5, pady=5)
        self.ang_velocity_label = tk.Label(self.velocity_frame, text="0 [deg/s]", bg='white', fg="black")
        self.ang_velocity_label.pack(side='right', fill='x', padx=5, pady=5)
        # WHEELS
        self.wheel_vel_frame = tk.Frame(self.data_frame, bg='white', borderwidth=2)
        self.wheel_vel_frame.pack(side='top', fill='x', expand=True, padx=5, pady=5)
        self.wheel_text = tk.Label(self.wheel_vel_frame, text="Wheels velocities: ", bg='white', fg="black")
        self.wheel_text.pack(side='top', fill='x', padx=5, pady=5)
        self.right_wheel_label = tk.Label(self.wheel_vel_frame, text="0 [rpm]", bg='white', fg="black")
        self.right_wheel_label.pack(side='left', fill='x', padx=5, pady=5)
        self.left_wheel_label = tk.Label(self.wheel_vel_frame, text="0 [rpm]", bg='white', fg="black")
        self.left_wheel_label.pack(side='right', fill='x', padx=5, pady=5)
        # POSITION
        self.position_frame = tk.Frame(self.data_frame, bg='white', borderwidth=2)
        self.position_frame.pack(side='top', fill='x', expand=True, padx=5, pady=5)
        self.position_text = tk.Label(self.position_frame, text="Position: ", bg='white', fg="black")
        self.position_text.pack(side='top', fill='x', padx=5, pady=5)
        self.x_pos_label = tk.Label(self.position_frame, text="X:  0 [cm]", bg='white', fg="black")
        self.x_pos_label.pack(side='top', fill='x', padx=5, pady=5)
        self.y_pos_label = tk.Label(self.position_frame, text="Y:  0 [cm]", bg='white', fg="black")
        self.y_pos_label.pack(side='top', fill='x', padx=5, pady=5)
        self.theta_pos_label = tk.Label(self.position_frame, text="T:  0 [deg]", bg='white', fg="black")
        self.theta_pos_label.pack(side='top', fill='x', padx=5, pady=5)

        self.control_frame = tk.Frame(self.manual_frame, bg='gray', borderwidth=2)
        self.control_frame.pack(side='bottom', fill='x', expand=True, padx=5, pady=5)
        controls_info = """Controls:\n   W - Forward\n   A - Left\n   S - Backward\n   D - Right"""
        self.controls_label = tk.Label(self.control_frame, text=controls_info, bg='gray', fg="white", justify='left')
        self.controls_label.pack(side='left', fill='y', padx=5, pady=5)


        # self.speed_bar.pack(side='right', fill='y', padx=5, pady=5)
        # self.increase_speed_button.pack(side='right', fill='y', padx=5, pady=5)

    def check_connection(self):
        # TEST CONNECTION
        is_alive = False
        if self.main_socket is not None:
            try:
                self.main_socket.send("PING".encode())
                s = self.main_socket.recv(1024)
                if s.decode() == "OK":
                    is_alive = True

            except (BrokenPipeError, ConnectionResetError, socket.error):
                self.main_socket = None
                is_alive = False

        if is_alive:
            self.status_label.config(text="Connected")
            self.connect_button.pack_forget()
            self.not_connected_frame.pack_forget()
            self.main_frame.pack()
        else:
            self.status_label.config(text="Not Connected")
            self.connect_button.pack(side='right', fill='y', padx=5, pady=10)
            self.not_connected_frame.pack()
            self.main_frame.pack_forget()

        self.after(3000, self.check_connection)


    def connect(self):
        try:
            self.main_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.main_socket.settimeout(5)
            self.main_socket.connect((HOSTNAME, MAIN_PORT))

        except (socket.timeout, ConnectionRefusedError):
            self.main_socket = None
            return 

        except socket.error:
            self.main_socket = None
            return 

        self.status_label.config(text="Connected")
        self.connect_button.pack_forget()
        self.not_connected_frame.pack_forget()
        self.main_frame.pack()

    def start_manual(self):
        self.main_frame.pack_forget()
        self.manual_frame.pack(side='top', fill='both', expand=True, padx=5, pady=5)



        # self.data_frame = tk.Frame(self.root, padx=10, pady=10, bg="lightblue")
        # self.data_frame.grid(row=0, column=0, sticky="nsew")

        # self.data_label = tk.Label(self.data_frame, text="Data will be displayed here", bg="lightblue")
        # self.data_label.pack()

        # # Keyboard Input Section
        # self.keyboard_frame = tk.Frame(self.root, padx=10, pady=10, bg="lightgreen")
        # self.keyboard_frame.grid(row=0, column=1, sticky="nsew")

        # self.direction_label = tk.Label(self.keyboard_frame, text="Direction: None", bg="lightgreen")
        # self.direction_label.pack()

        # self.root.bind("<Key>", self.key_pressed)


if __name__ == "__main__":
    gui = RobotGUI()
    gui.mainloop()

