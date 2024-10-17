import tkinter as tk
from ..components.camera_frame import CameraFrame

class MainPage(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.config(borderwidth=1, relief='solid')
    
        self.title = tk.Label(self, text="Robot Controller", font=("Arial", 24))
        self.status = tk.Label(self, text="Connected", font=("Arial", 18), fg='lightgray')

        self.camera_frame = CameraFrame(self)

        self.side_frame = tk.Frame(self)
        self.manual_button = tk.Button(self.side_frame, text="Manual Control", font=("Arial", 16))
        self.settings_button = tk.Button(self.side_frame, text="Settings", font=("Arial", 16))


        self.title.pack(side='top', pady=20)    
        self.status.pack(side='top', pady=0)
        self.camera_frame.pack(side='left', fill='both', expand=True, padx=70, pady=40)
        self.side_frame.pack(side='right', pady=20)
        self.manual_button.pack(side='top', pady=20)
        self.settings_button.pack(side='top', pady=20)


    def show(self):
        self.pack(fill='both', expand=True)

    def hide(self):
        self.pack_forget()
    