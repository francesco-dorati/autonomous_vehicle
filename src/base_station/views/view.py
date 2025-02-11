import tkinter as tk
from .sidebar import Sidebar

# from .main_page.not_connected_page import NotConnectedPage
from .main_page import MainPage
from .manual_page.manual_page import ManualPage
from .settings_page.settings_page import SettingsPage


class View(tk.Frame):
    def __init__(self, controller, master=None):
        super().__init__(master)
        self.pack(fill='both', expand=True)

        self.controller = controller
        self.controller.set_view(self)
        
        self.sidebar = Sidebar(self, controller)
        self.sidebar.show()

        self.main_page = MainPage(self, controller)
        self.main_page.show()

    # def show_page(self, page_name):
    #     self.current_page = self.pages[page_name]
    #     for page in self.pages.values():
    #         page.hide()
    #     self.pages[page_name].show()
    #     self.update_idletasks()

    def connect(self):
        self.sidebar.connect()
        self.main_page.connect()
        
    def disconnect(self):
        self.sidebar.disconnect()
        self.main_page.disconnect()
        
    