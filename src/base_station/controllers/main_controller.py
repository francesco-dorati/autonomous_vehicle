import time
import socket

from controllers.manual_controller import ManualController

PING_INTERVAL_MS = 8000

class MainController:
    def __init__(self, main_view):
        self.main_view = main_view
        self.server_address = ('172.20.10.3', 5500)
        self.main_connection = None

        self.manual_controller = None

        self.add_commands()
        self.check_connection()
    
    def add_commands(self):
        # SIDEBAR
        self.main_view.sidebar.connect_button.config(command=self.connect)
        self.main_view.sidebar.disconnect_button.config(command=self.disconnect)
        self.main_view.sidebar.confirm_button.config(command=self.confirm_config_sidebar)
        self.main_view.sidebar.hostname.insert(0, self.server_address[0])
        self.main_view.sidebar.port.insert(0, self.server_address[1])
        self.main_view.sidebar.exit_button.config(command=self.exit)

        # MAIN PAGE
        self.main_view.pages['main'].manual_button.config(command=self.start_manual)

        # # MANUAL PAGE
        self.main_view.pages['manual'].exit_button.config(command=self.exit_manual)

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
    
    def disconnect(self):
        # disconnect logic
        if self.main_connection:
            self.main_connection.send(b"E")
            self.main_connection.close()
        self.main_connection = None
        self.main_view.disconnect()

    def check_connection(self):
        if self.main_connection:
            try:
                t_start = time.time()
                self.main_connection.send(b'P')
                response = self.main_connection.recv(32)
                dt_ms = (time.time() - t_start)*1000
                response = response.decode().strip().split(' ')
                if response[0] == "P":
                    battery_voltage = float(response[1])
                    self.main_view.sidebar.update_ping(dt_ms, battery_voltage)
                else:
                    self.disconnect()

            except (socket.timeout, BrokenPipeError, ConnectionResetError, socket.error):
                self.disconnect()

            # ping server
            # if receaves response, keep connection
            # else disconnect
            
        self.main_view.after(PING_INTERVAL_MS, self.check_connection)

    def confirm_config_sidebar(self):
        if self.main_connection:
            return 
        
        self.server_address = (self.main_view.sidebar.hostname.get(), int(self.main_view.sidebar.port.get()))
        self.main_view.sidebar.close_config()


    def start_manual(self):
        if not self.main_connection:
            return
        
        self.main_view.show_page('manual')
        self.manual_controller = ManualController(self.main_view, self.main_view.current_page, self.main_connection)
        # self.manual_controller.start(self.main_connection)

    
    def exit_manual(self):
        self.manual_controller.stop()
        self.main_view.show_page('main')

   

