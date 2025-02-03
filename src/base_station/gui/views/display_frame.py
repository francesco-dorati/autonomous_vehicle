import tkinter as tk
class DisplayFrame(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.config(borderwidth=2, relief='raised', bg='lightgray', width=400, height=400)
        
        self.controller = controller

    def show(self):
        self.pack(side='top', fill='both', expand=True, pady=10)
        
        #pack(side='top', fill='both', expand=True, pady=10)
