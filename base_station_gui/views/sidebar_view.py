import tkinter as tk
from tkinter import ttk

class SidebarView(tk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.config(width=100, borderwidth=1, relief='solid')
        self.pack(side='left', fill='y')

        # MAIN FRAME
        self.connection_frame = tk.Frame(self)
        
        self.status_frame = tk.Frame(self.connection_frame)
        self.status_label = tk.Label(self.status_frame, text="Status: ", font=("Arial", 16))
        self.status = tk.Label(self.status_frame, text="✕ Not Connected", font=("Arial", 16))  
        self.ping_label = tk.Label(self.status_frame, text="Ping: ", font=("Arial", 16))
        self.ping = tk.Label(self.status_frame, text="-", font=("Arial", 16))
        
        self.connect_button = tk.Button(self.connection_frame, text="Connect", font=("Arial", 16))
        self.disconnect_button = tk.Button(self.connection_frame, text="Disconnect", font=("Arial", 16))
        
        # CONFIG
        self.config_button = tk.Button(self.connection_frame, text="Connection Config", font=("Arial", 16), command=self.open_config)
        self.config_frame = tk.Frame(self.connection_frame)
        self.hostname_label = ttk.Label(self.config_frame, text="Hostname: ")
        self.hostname = ttk.Entry(self.config_frame)
        self.port_label = ttk.Label(self.config_frame, text="Port: ")
        self.port = ttk.Entry(self.config_frame)
        self.confirm_button = tk.Button(self.config_frame, text="Confirm", font=("Arial", 16), width=25)
        self.cancel_button = tk.Button(self.config_frame, text="Cancel", font=("Arial", 16), command=self.close_config, width=25)


        # LOWER FRAME
        self.lower_frame = tk.Frame(self)
        self.exit_button = tk.Button(self.lower_frame, text="Exit...", font=("Arial", 16))
        self.shutdown_button = tk.Button(self.lower_frame, text="Shutdown Robot", font=("Arial", 16))


        # PACKING
        self.connection_frame.pack(fill='both', side='top', padx=0, pady=0)
        self.status_frame.pack(fill='x', side='top', pady=30, padx=30)
        self.status_label.grid(row=0, column=0, sticky=tk.W, padx=10, pady=5)
        self.status.grid(row=0, column=1, padx=10, pady=5)
        # self.status_label.pack(fill='y', side='top')
        # self.status.pack(fill='y', side='top')


        self.connect_button.pack(fill='x', side='top', pady=3, padx=10)
        self.config_button.pack(fill='x', side='bottom', pady=3, padx=10)

        self.hostname_label.grid(row=0, column=0, sticky=tk.W, padx=10, pady=5)
        self.hostname.grid(row=0, column=1, padx=10, pady=5)
        self.port_label.grid(row=1, column=0, sticky=tk.W, padx=10, pady=5)
        self.port.grid(row=1, column=1, padx=10, pady=5)
        self.confirm_button.grid(row=2, column=0, columnspan=2, padx=10, pady=5)
        self.cancel_button.grid(row=3, column=0, columnspan=2, padx=10, pady=5)


        self.lower_frame.pack(fill='x', side='bottom')
        self.exit_button.pack(fill='x', side='bottom', pady=10)

    def connect(self):
        self.status.config(text="✓ Connected")
        self.connect_button.pack_forget()
        self.config_button.pack_forget()
        self.disconnect_button.pack(fill='x', side='top', pady=10)
        self.ping_label.grid(row=1, column=0, sticky=tk.W, padx=10, pady=5)
        self.ping.grid(row=1, column=1, padx=10, pady=5)
        self.shutdown_button.pack(fill='x', side='bottom', pady=10)
    
    def connecting(self):
        self.status.config(text="Connecting...")
        self.connect_button.pack_forget()

    def disconnect(self):
        self.status.config(text="✕ Not Connected")
        self.disconnect_button.pack_forget()
        self.connect_button.pack(fill='x', side='top', pady=10)
        self.config_button.pack(fill='x', side='top', pady=3, padx=10)
        self.ping_label.grid_forget()
        self.ping.grid_forget()
        self.shutdown_button.pack_forget()
    
    def update_ping(self, ping):
        self.ping.config(text=f"{ping:.2f} ms")

    def open_config(self):
        self.config_frame.pack(fill='x', side='top', pady=10)
        self.config_button.pack_forget()

    def close_config(self):
        self.config_button.pack(fill='x', side='top', pady=3, padx=10)
        self.config_frame.pack_forget()


