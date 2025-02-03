from tkinter import Tk
from models.model import Model
from views.view import View
from controllers.main_controller import Controller


class RobotApp(Tk):
    def __init__(self):
        super().__init__()
        self.title("Robot Control Panel")
        self.geometry("1200x750")

        self.model = Model()
        self.controller = Controller(self.model)
        self.view = View(self.controller)


if __name__ == "__main__":
    app = RobotApp()
    app.mainloop()
