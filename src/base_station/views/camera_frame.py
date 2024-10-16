import tkinter as tk 
from PIL import Image, ImageTk

class CameraFrame(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.config(borderwidth=2, bg='lightgray', relief='solid', width=400, height=400)

        self.start_button = tk.Button(self, text="Start Camera", font=("Arial", 16))
        self.start_button.pack(fill='none', pady=20, expand=True)

        self.stop_button = tk.Button(self, text="Stop Camera", font=("Arial", 16))

        self.image_label = tk.Label(self)
    
    def start(self):
        self.start_button.pack_forget()
        self.image_label.pack()
        self.stop_button.place(relx=0.5, rely=0.9, anchor='s')  # Place button at the bottom center over the image
        self.stop_button.lift()

    def disable(self):
        self.image_label.pack_forget()
        self.stop_button.pack_forget()
        self.start_button.pack(fill='none', pady=20, expand=True)

    def update_image(self, image):
        imgtk = ImageTk.PhotoImage(image=image)
        self.image_label.imgtk = imgtk
        self.image_label.configure(image=imgtk)