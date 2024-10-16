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

    # Create sensor lines (bars) on the top and bottom
    bar_height = 10
    max_distance = 150  # Maximum distance for scaling the bars
    sensor_positions_top = [120, 200, 280]  # X positions for the top bars
    sensor_positions_bottom = [120, 200, 280]  # X positions for the bottom bars

    # Draw the top bars
    for index, x in enumerate(sensor_positions_top):
        distance = distances[index]
        # Calculate bar length based on distance
        bar_length = (distance / max_distance) * 100  # Scale to a maximum length
        y_top = 140  # Y position for the top bar
        canvas.create_rectangle(x - bar_length // 2, y_top,
                                x + bar_length // 2, y_top + bar_height,
                                fill='blue')

    # Draw the bottom bars
    for index, x in enumerate(sensor_positions_bottom):
        distance = distances[index + 3]  # Assuming you have a separate distance list for bottom sensors
        # Calculate bar length based on distance
        bar_length = (distance / max_distance) * 100  # Scale to a maximum length
        y_bottom = 360  # Y position for the bottom bar
        canvas.create_rectangle(x - bar_length // 2, y_bottom,
                                x + bar_length // 2, y_bottom + bar_height,
                                fill='blue')

    root.mainloop()

# Example distances for the sensors (in pixels or arbitrary units)
# Top sensors: [30, 70, 120]
# Bottom sensors: [60, 80, 140]
sensor_distances = [30, 70, 120, 60, 80, 140]  # Example distances for each sensor
create_simulation_window(sensor_distances)