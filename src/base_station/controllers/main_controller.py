import time
import socket
import pygame

from base_station.controllers.manual_controller import ManualController
from base_station.controllers.input_handler  import InputHandler
from base_station.network import ClientConnection, DataReceiver, ManualTransmitter
from base_station.config import SERVER_HOST, MAIN_PORT, DATA_PORT


PING_INTERVAL_MS = 5000
class Controller:
    def __init__(self):
        self.view = None
        # self.model = model
        self.connection = ClientConnection(SERVER_HOST, MAIN_PORT)
        self.data_receiver = DataReceiver(DATA_PORT, self.update_data)
        self.manual_transmitter = None
        self.input_handler = None

    def set_view(self, view):
        self.view = view

    def connect(self):
        ok = self.connection.connect()
        if ok:
            self.data_receiver.start()
            self.__periodic_ping()
            self.view.connect()
        
    def disconnect(self):
        # TODO disconnect all
        self.data_receiver.stop()
        self.connection.disconnect()
        self.view.disconnect()

    # MAP
    def new_map(self, entry):
        name = entry.get().strip()
        self.connection.new_global_map(name)
        # self.model.new_global_map(name)
        self.view.main_page.set_map(self.model.global_map_name)
    
    def discard_map(self):
        self.model.discard_global_map()
        self.view.main_page.discard_map()
    
    # CONTROL
    def set_control(self, control_type: str):
        if control_type == "manual":
            # Start manual control
            ok, port = self.connection.start_manual_control()
            if ok:
                manual_frame = self.view.main_page.controls_frame.manual_frame
                self.input_handler = InputHandler(self, manual_frame)
                self.input_handler.set_input("keyboard")
                self.manual_transmitter = ManualTransmitter(SERVER_HOST, port, self.input_handler)
                self.manual_transmitter.start()

        elif control_type == "off":
            if self.manual_transmitter:
                self.manual_transmitter.stop()
                self.manual_transmitter = None
            if self.input_handler:
                self.input_handler.set_input("off")
                self.input_handler = None
            self.connection.stop_control()
        
        self.view.main_page.set_control(control_type)

    def update_data(self, size, global_map, lidar_points, robot_pos):
        # update map
        if self.view and self.view.main_page.display:
            self.view.main_page.display.update(size, global_map, lidar_points, robot_pos)


    # def set_manual_control(self, _type: str):
    #     if self.model.control_type == "manual":
    #         self.model.manual_control_type = _type
    #         if _type == "keyboard":
    #             self.stop_joypad_reading()
    #             self.view.main_page.controls_frame.manual_frame.hide_joypad()
    #             self.view.main_page.controls_frame.manual_frame.show_keyboard()
    #             self.view.update_idletasks()
    #             self.start_keyboard_reading()
    #         elif _type == "joypad":
    #             self.stop_keyboard_reading()
    #             self.view.main_page.controls_frame.manual_frame.hide_keyboard()
    #             self.view.main_page.controls_frame.manual_frame.show_joypad()
    #             self.view.update_idletasks()
    #             self.start_joypad_reading()
        
    def start_keyboard_reading(self):
        if self.model.control_type == "manual":
            manual_frame = self.view.main_page.controls_frame.manual_frame
            manual_frame.focus_set()
            manual_frame.bind("<KeyPress>", self.key_event)
            manual_frame.bind("<KeyRelease>", self.key_event)

    def stop_keyboard_reading(self):
            manual_frame = self.view.main_page.controls_frame.manual_frame
            manual_frame.focus_set()
            manual_frame.bind("<KeyPress>", self.key_event)
            manual_frame.bind("<KeyRelease>", self.key_event)
            self.model.keyboard_buffer = {'x': 0, 'y': 0}

    def start_joypad_reading(self):
        pygame.init()
        pygame.joystick.init()
        self.joypad_loop()

    def stop_joypad_reading(self):
        pygame.joystick.quit()
        pygame.quit()
        self.model.joypad = None
        self.model.joypad_buffer = {'x': 0, 'y': 0}


    def key_event(self, event):
        key = event.keysym
        if key.lower() in ['w', 's', 'a', 'd']:
            if key.lower() == 'w':
                if event.type == "2":
                    self.model.keyboard_buffer['x'] = min(self.model.keyboard_buffer['x']+1, 1)
                elif event.type == "3":
                    self.model.keyboard_buffer['x'] = max(self.model.keyboard_buffer['x']-1, -1)
            elif key == 's':
                if event.type == "2":
                    self.model.keyboard_buffer['x'] = max(self.model.keyboard_buffer['x']-1, -1)
                elif event.type == "3":
                    self.model.keyboard_buffer['x'] = min(self.model.keyboard_buffer['x']+1, 1)
            elif key == 'a':
                if event.type == "2":
                    self.model.keyboard_buffer['y'] = min(self.model.keyboard_buffer['y']+1, 1)
                elif event.type == "3":
                    self.model.keyboard_buffer['y'] = max(self.model.keyboard_buffer['y']-1, -1)
            elif key == 'd':
                if event.type == "2":
                    self.model.keyboard_buffer['y'] = max(self.model.keyboard_buffer['y']-1, -1)
                elif event.type == "3":
                    self.model.keyboard_buffer['y'] = min(self.model.keyboard_buffer['y']+1, 1)
            
            self.view.main_page.controls_frame.manual_frame.set_key(self.model.keyboard_buffer)

    def joypad_loop(self):
        if self.model.control_type == "manual" and self.model.manual_control_type == "joypad":
            pygame.event.pump()
            if self.model.joypad:
                # print(self.model.joypad)
                self.model.joypad_buffer['x'] = float(-self.model.joypad.get_axis(1))
                self.model.joypad_buffer['y'] = float(-self.model.joypad.get_axis(0))
                self.view.main_page.controls_frame.manual_frame.set_joypad(self.model.joypad_buffer)
            else:
                if pygame.joystick.get_count() > 0:
                    self.model.joypad = pygame.joystick.Joystick(0)
                    self.model.joypad.init()
                    self.view.main_page.controls_frame.manual_frame.connect_joypad()
                else:
                    self.model.joypad = None
                    self.view.main_page.controls_frame.manual_frame.disconnect_joypad()

            self.view.after(100, self.joypad_loop)


    def __periodic_ping(self):
        if self.model.main_connection:
            try:
                ok, update_control, update_map = self.connection.ping()
                if ok:
                    # update sidebar
                    self.view.sidebar.update_ping(self.model.battery_V, self.model.ping_time)
                    if update_control:
                        self.view.main_page.set_control(self.model.control_type)
                    if update_map:
                        self.view.main_page.set_map(self.model.global_map_name)
                else:
                    self.disconnect()
                    return

            except (socket.timeout, BrokenPipeError, ConnectionResetError, socket.error):
                self.disconnect()
                return

            self.view.after(PING_INTERVAL_MS, self.__periodic_ping)
        
    # def start_manual_control(self):
    #     self.view.main_page.controls_frame.show_manual()
        
    # def start_auto_control(self):
    #     pass

    # def stop_control(self):
    #     self.view.main_page.controls_frame.show_choice()
        

class MainController2:
    def __init__(self, main_view):
        self.main_view = main_view
        self.server_address = ('172.20.10.3', 5500)
        self.main_connection = None

        self.manual_controller = None

        self.add_commands()
    
    
    def add_commands(self):
        # SIDEBAR
        self.main_view.sidebar.connect_button.config(command=self.connect)
        self.main_view.sidebar.disconnect_button.config(command=self.disconnect)
        self.main_view.sidebar.confirm_button.config(command=self.confirm_config_sidebar)
        self.main_view.sidebar.hostname.insert(0, self.server_address[0])
        self.main_view.sidebar.port.insert(0, self.server_address[1])
        self.main_view.sidebar.exit_button.config(command=self.exit)

        # MAIN PAGE
        self.main_view.pages['main'].manual_button.config(command=self.open_manual)

        # # MANUAL PAGE
        # self.main_view.pages['manual'].exit_button.config(command=self.exit_manual)

    def exit(self):
        if self.main_connection:
            self.disconnect()

            if self.main_view.current_page:
            # send exit command to server
                pass
        exit(0)

    def connect(self):
        self.main_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.main_connection.settimeout(3)
        try:
            self.main_connection.connect(self.server_address)
        except (socket.timeout, ConnectionRefusedError):
            self.main_connection = None
            return
        self.main_view.connect()
        self.check_connection()

    
    def disconnect(self):
        # disconnect logic
        if self.manual_controller and self.manual_controller.running:
            self.manual_controller.stop()
            self.manual_controller = None
            time.sleep(0.1)
        if self.main_connection:
            self.main_connection.send(b"E\n")
            self.main_connection.close()
        self.main_connection = None
        self.main_view.disconnect()

    def check_connection(self):
        if self.main_connection:
            try:
                t_start = time.time()
                self.main_connection.send(b'P\n')
                response = self.main_connection.recv(32)
                dt_ms = (time.time() - t_start)*1000
                response = response.decode().strip().split(' ')
                if response[0] == "P":
                    battery_voltage = float(response[1])
                    battery_level = response[2]
                    self.main_view.sidebar.update_ping(dt_ms, battery_voltage, battery_level)
                else:
                    self.disconnect()

            except (socket.timeout, BrokenPipeError, ConnectionResetError, socket.error):
                self.disconnect()

            self.main_view.after(PING_INTERVAL_MS, self.check_connection)
            # ping server
            # if receaves response, keep connection
            # else disconnect
            

    def confirm_config_sidebar(self):
        if self.main_connection:
            return 
        
        self.server_address = (self.main_view.sidebar.hostname.get(), int(self.main_view.sidebar.port.get()))
        self.main_view.sidebar.close_config()


    def open_manual(self):
        if not self.main_connection:
            return
        
        self.main_view.show_page('manual')
        self.manual_controller = ManualController(self.main_view, self.main_view.current_page, self.main_connection)
        

    def exit_manual(self):
        self.manual_controller.stop()
        self.main_view.show_page('main')

   

