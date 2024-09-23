import tkinter as tk
from views.sidebar import Sidebar

from views.not_connected_page import NotConnectedPage
from views.main_page import MainPage
from views.manual_page import ManualPage
from views.settings_page import SettingsPage


class View(tk.Frame):
    def __init__(self, master):
        super().__init__(master)
        self.pack(fill='both', expand=True)

        self.sidebar = Sidebar(self)

        self.pages = {
            'not_connected': NotConnectedPage(self),
            'main': MainPage(self),
            'manual': ManualPage(self), 
            'settings': SettingsPage(self),
        }
        self.show_page('not_connected')

    def show_page(self, page_name):
        self.current_page = self.pages[page_name]
        for page in self.pages.values():
            page.hide()
        self.pages[page_name].show()
        self.update_idletasks()
    
    def connect(self):
        self.sidebar.connect()
        self.show_page('main')
        self.current_page.connect()
    
    def disconnect(self):
        self.sidebar.disconnect()
        self.show_page('not_connected')
        self.current_page.disconnect()
    