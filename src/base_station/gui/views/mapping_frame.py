import tkinter as tk
class MappingFrame(tk.Frame):
    def __init__(self, parent, controller):
        super().__init__(parent)
        self.config(borderwidth=2, relief='raised')

        self.controller = controller

        self.text_frame = tk.Frame(self) 
        self.map_label = tk.Label(self.text_frame, text="Map: ", font=("Arial", 15, 'bold'))
        self.map_name = tk.Label(self.text_frame, text="---")
        self.mapping_checkbox = tk.Checkbutton(self.text_frame, text=" mapping", font=("Arial", 14))

        self.map_frame = tk.Frame(self, borderwidth=2, relief='raised', bg='lightgray', width=200, height=200)

        self.save_button = tk.Button(self, text="Save Map", font=("Arial", 15))
        self.discard_button = tk.Button(self, text="Discard Map", font=("Arial", 15), command=self.controller.discard_map)

        self.new_map_button = tk.Button(self, text="New Map", font=("Arial", 15), command=self.new_map_popup)
        self.load_map_button = tk.Button(self, text="Load Map", font=("Arial", 15))
        self.popup = None
        
    def show(self):
        self.pack(side='top', fill='x')
        self.text_frame.pack(side='top', fill='x', pady=10)
        self.map_label.grid(row=0, column=0, sticky=tk.W, padx=10, pady=5)
        self.map_name.grid(row=0, column=1, sticky=tk.W, padx=10, pady=5)
        self.mapping_checkbox.grid(row=1, column=0, padx=10, pady=5)
        self.map_frame.pack(side='top', fill='none', pady=10, padx=10)
        self.new_map_button.pack(side='top', fill='x', padx=10, pady=5)
        self.load_map_button.pack(side='top', fill='x', padx=10, pady=5)
        self.disable()
    
    def enable(self):
        """ enable for user actions """
        self.map_label.config(state='normal')
        self.map_name.config(state="normal")
        self.save_button.config(state='normal')
        self.discard_button.config(state='normal')
        self.new_map_button.config(state='normal')
        self.load_map_button.config(state='normal')

    def disable(self):
        """ disable from user interaction"""
        self.map_label.config(state='disabled')
        self.map_name.config(text="---", state="disabled")
        self.mapping_checkbox.config(state='disabled')
        self.map_frame.config(bg='lightgray')
        self.save_button.config(state='disabled')
        self.discard_button.config(state='disabled')
        self.new_map_button.config(state='disabled')
        self.load_map_button.config(state='disabled')
    
    def set_map(self, map_name):
        """ setup new map """
        self.map_name.config(text=map_name)
        self.mapping_checkbox.config(state='normal')
        if self.popup:
            self.popup.destroy()

        self.new_map_button.pack_forget()
        self.load_map_button.pack_forget()
        self.save_button.pack(side='top', fill='x', padx=10, pady=5)
        self.discard_button.pack(side='top', fill='x', padx=10, pady=5)
    
    def discard_map(self):
        """ discard the current map """
        self.map_name.config(text="---")
        self.mapping_checkbox.config(state='disabled')
        self.save_button.pack_forget()
        self.discard_button.pack_forget()
        self.new_map_button.pack(side='top', fill='x', padx=10, pady=5)
        self.load_map_button.pack(side='top', fill='x', padx=10, pady=5)
    
    def new_map_popup(self):
        self.popup = tk.Toplevel(self)
        self.popup.title("New Map")
        self.popup.geometry("300x200")
        self.popup.resizable(False, False)
        self.popup.grab_set()
        self.popup.transient(self)
        self.popup.focus_force()
        tk.Label(self.popup, text="Enter map name:", anchor="w").pack(fill="x")
        entry = tk.Entry(self.popup)
        entry.pack(fill="x")
        tk.Button(self.popup, text="create", command=lambda: self.controller.new_map(entry)).pack()
 