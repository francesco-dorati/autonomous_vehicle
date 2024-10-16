import tkinter as tk

# Function to create the simulation window
def create_simulation_window(distances):
    root = tk.Tk()
    root.title("Distance Sensors Simulation")

    # Set canvas size
    canvas_width = 600
    canvas_height = 400
    canvas = tk.Canvas(root, width=canvas_width, height=canvas_height)
    canvas.pack()

    # Create a larger rectangle (car) rotated 90 degrees
    car_rectangle = canvas.create_rectangle(100, 150, 300, 350, fill='lightgray')

    # Coordinates for the sensor squares (now inside the rectangle)
    sensor_positions = [
        (120, 170),  # Top-left
        (280, 170),  # Top-right
        (120, 330),  # Bottom-left
        (280, 330)   # Bottom-right
    ]

    # Create sensor squares
    sensor_size = 40  # Size of the sensor squares

    for index, (x, y) in enumerate(sensor_positions):
        # Choose color based on distance
        distance = distances[index]
        if distance < 50:
            color = 'red'
            text_color = 'white'
        elif distance < 100:
            color = 'orange'
            text_color = 'black'
        elif distance < 150:
            color = 'yellow'
            text_color = 'black'
        else:
            color = 'green'
            text_color = 'black'

        # Draw the sensor square, ensuring they stay within the rectangle
        canvas.create_rectangle(x - sensor_size // 2, y - sensor_size // 2,
                                x + sensor_size // 2, y + sensor_size // 2,
                                fill=color)

        # Draw the distance text inside the square
        canvas.create_text(x, y, text=str(distance), font=("Arial", 12), fill=text_color)

    root.mainloop()

# Example distances for the sensors (in pixels or arbitrary units)
sensor_distances = [30, 70, 120, 200]  # Example distances for each sensor
create_simulation_window(sensor_distances)