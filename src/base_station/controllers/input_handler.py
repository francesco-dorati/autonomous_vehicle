import time
import pygame
import threading

class InputHandler:
    def __init__(self, manual_frame):
        # self.controller = controller
        self.manual_frame = manual_frame
        self.buffer = {'x': 0, 'y': 0}  # Unified buffer for keyboard & joypad
        self.input_type = None
        self.joypad = None

    def set_input(self, input_type):
        print("Input type set to", input_type)
        self.input_type = input_type
        self.buffer = {'x': 0, 'y': 0}
        if input_type == "keyboard":
            self.__stop_joypad_reading()
            self.manual_frame.hide_joypad()
            self.manual_frame.show_keyboard()
            self.__start_keyboard_reading()
        elif input_type == "joypad":
            self.__stop_keyboard_reading()
            self.manual_frame.hide_keyboard()
            self.manual_frame.show_joypad()
            self.__start_joypad_reading()
        else:
            self.__stop_keyboard_reading()
            self.__stop_joypad_reading()
            self.manual_frame.hide_keyboard()
            self.manual_frame.hide_joypad()
        self.manual_frame.update_idletasks()

    def get_buffer(self):
        if self.input_type == "joypad":
            self.update_joypad()
        return self.buffer
    
    def __start_keyboard_reading(self):
        self.manual_frame.focus_set()
        self.manual_frame.bind("<KeyPress>", self.key_event)
        self.manual_frame.bind("<KeyRelease>", self.key_event)
    def __stop_keyboard_reading(self):
        self.manual_frame.unbind("<KeyPress>")
        self.manual_frame.unbind("<KeyRelease>")
    def __start_joypad_reading(self):
        pygame.init()
        pygame.joystick.init()
    def __stop_joypad_reading(self):
        pygame.joystick.quit()
  

    def key_event(self, event):
        """Handle keyboard input and update command buffer"""
        key = event.keysym
        if key.lower() in ['w', 's', 'a', 'd'] and self.input_type == "keyboard":
            if key.lower() == 'w':
                if event.type == "2":
                    self.buffer['x'] = min(self.buffer['x']+1, 1)
                elif event.type == "3":
                    self.buffer['x'] = max(self.buffer['x']-1, -1)
            elif key == 's':
                if event.type == "2":
                    self.buffer['x'] = max(self.buffer['x']-1, -1)
                elif event.type == "3":
                    self.buffer['x'] = min(self.buffer['x']+1, 1)
            elif key == 'a':
                if event.type == "2":
                    self.buffer['y'] = min(self.buffer['y']+1, 1)
                elif event.type == "3":
                    self.buffer['y'] = max(self.buffer['y']-1, -1)
            elif key == 'd':
                if event.type == "2":
                    self.buffer['y'] = max(self.buffer['y']-1, -1)
                elif event.type == "3":
                    self.buffer['y'] = min(self.buffer['y']+1, 1)
            self.manual_frame.set_key(self.buffer)

    def update_joypad(self):
        # Schedule this update on the main thread
        if threading.current_thread() != threading.main_thread():
            self.manual_frame.after(0, self.update_joypad)
            return
        
        if self.input_type == "joypad":
            print("Updating joypad")
            pygame.event.pump()
            if self.joypad:
                # print(self.model.joypad)
                self.buffer['x'] = float(-self.joypad.get_axis(1))
                self.buffer['y'] = float(-self.joypad.get_axis(0))
                self.manual_frame.set_joypad(self.buffer)
            else:
                if pygame.joystick.get_count() > 0:
                    self.joypad = pygame.joystick.Joystick(0)
                    self.joypad.init()
                    self.manual_frame.connect_joypad()
                else:
                    self.joypad = None
                    self.manual_frame.disconnect_joypad()

