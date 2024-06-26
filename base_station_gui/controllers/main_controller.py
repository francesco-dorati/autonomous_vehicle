import time

CONNECTION_TIMEOUT_MS = 3000

class MainController:
    def __init__(self, main_view):
        self.main_view = main_view
        self.server_address = ('172.20.10.7', 5500)

        self.main_connection = None

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

        # MANUAL PAGE
        self.main_view.pages['manual'].exit_button.config(command=self.exit_manual)

    def exit(self):
        if self.main_connection:
            # send exit command to server
            pass
        exit(0)

    def connect(self):
        # connect logic
        self.main_connection = True
        self.main_view.connect()
    
    def disconnect(self):
        # disconnect logic
        self.main_connection = None
        self.main_view.disconnect()

    def check_connection(self):
        if self.main_connection:
            # ping server
            # if receaves response, keep connection
            # else disconnect
            pass
        self.main_view.after(CONNECTION_TIMEOUT_MS, self.check_connection)

    def confirm_config_sidebar(self):
        if self.main_connection:
            return 
        
        self.server_address = (self.main_view.sidebar.hostname.get(), int(self.main_view.sidebar.port.get()))
        self.main_view.sidebar.close_config()


    def start_manual(self):
        if not self.main_connection:
            return
        
        self.main_view.show_page('manual')
        self.start_keyboard_control()
    
    def exit_manual(self):
        self.main_view.show_page('main')
        self.stop_keyboard_control()

    def start_keyboard_control(self):
        self.main_view.focus_set()
        self.main_view.bind("<KeyPress>", self.key_pressed)
        self.main_view.bind("<KeyRelease>", self.key_released)
        # self.main_view.bind("<KeyPress>", (lambda event: self.key_pressed(event, True)))
        # self.main_view.bind("<KeyRelease>", (lambda event: self.key_pressed(event, False)))

    def stop_keyboard_control(self):
        self.main_view.unbind("<KeyPress>")
        self.main_view.unbind("<KeyRelease>")

    def key_pressed(self, event):
        key = event.keysym
        if key == 'w':
            self.main_view.pages['manual'].data_frame.forward(True)
        elif key == 's':
            self.main_view.pages['manual'].data_frame.backward(True)
        elif key == 'a':
            self.main_view.pages['manual'].data_frame.left(True)
        elif key == 'd':
            self.main_view.pages['manual'].data_frame.right(True)

    def key_released(self, event):
        key = event.keysym
        if key == 'w':
            self.main_view.pages['manual'].data_frame.forward(False)
        elif key == 's':
            self.main_view.pages['manual'].data_frame.backward(False)
        elif key == 'a':
            self.main_view.pages['manual'].data_frame.left(False)
        elif key == 'd':
            self.main_view.pages['manual'].data_frame.right(False)


