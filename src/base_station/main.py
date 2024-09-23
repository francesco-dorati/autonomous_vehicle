from tkinter import Tk
from views.view import View
from controllers.main_controller import MainController


class RobotApp(Tk):
    def __init__(self):
        super().__init__()
        self.title("Robot Control Panel")
        self.geometry("1200x750")

        self.main_view = MainView(self)
        self.main_view.pack()
        self.main_controller = MainController(self.main_view)

if __name__ == "__main__":
    app = RobotApp()
    app.mainloop()
