import tkinter as tk

class MainPage(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.config(borderwidth=1, relief='solid')
    
        self.title = tk.Label(self, text="Robot Controller", font=("Arial", 24))
        self.status = tk.Label(self, text="Not Connected", font=("Arial", 18), fg='lightgray')
        
        self.connected_frame = tk.Frame(self)
        self.manual_button = tk.Button(self.connected_frame, text="Manual Control", font=("Arial", 16))

        self.title.pack(side='top', pady=20)    
        self.status.pack(side='top', pady=0)
        self.manual_button.pack(side='top', pady=20)


    def show(self):
        self.pack(fill='both', expand=True)

    def hide(self):
        self.pack_forget()

    def connect(self):
        self.status.config(text="Connected", fg='lightgray')
        self.connected_frame.pack(side='top', pady=30)

    def disconnect(self):
        self.status.config(text="Not Connected", fg='lightgray')
        self.connected_frame.pack_forget()