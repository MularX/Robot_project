import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener, Buffer
import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()

        # Set up Transform Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robot_position = [0.0, 0.0]

        # Timer to update the position
        self.create_timer(0.1, self.update_position)

    def move_forward(self):
        self.twist.linear.x = 0.5
        self.twist.angular.z = 0.0
        self.publisher_.publish(self.twist)

    def move_backward(self):
        self.twist.linear.x = -0.5
        self.twist.angular.z = 0.0
        self.publisher_.publish(self.twist)

    def turn_left(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.5
        self.publisher_.publish(self.twist)

    def turn_right(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = -0.5
        self.publisher_.publish(self.twist)

    def stop_robot(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher_.publish(self.twist)

    def update_position(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.robot_position[0] = transform.transform.translation.x
            self.robot_position[1] = transform.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {e}")

def create_gui(controller):
    def on_forward():
        controller.move_forward()

    def on_backward():
        controller.move_backward()

    def on_left():
        controller.turn_left()

    def on_right():
        controller.turn_right()

    def on_stop(event):
        controller.stop_robot()

    # Create the main window
    window = tk.Tk()
    window.title("Robot Teleop GUI with Map")

    # Create buttons
    forward_button = tk.Button(window, text="↑", command=on_forward, width=10, height=2)
    backward_button = tk.Button(window, text="↓", command=on_backward, width=10, height=2)
    left_button = tk.Button(window, text="←", command=on_left, width=10, height=2)
    right_button = tk.Button(window, text="→", command=on_right, width=10, height=2)

    # Layout the buttons in a grid
    forward_button.grid(row=0, column=1)
    left_button.grid(row=1, column=0)
    right_button.grid(row=1, column=2)
    backward_button.grid(row=2, column=1)

    # Create a Figure for the map
    fig = Figure(figsize=(5, 5))
    ax = fig.add_subplot(111)
    ax.set_xlim(-10, 10)  # Set the map limits
    ax.set_ylim(-10, 10)
    robot_marker, = ax.plot(0, 0, 'ro')  # Robot position marker

    # Embed the Figure in the tkinter window
    canvas = FigureCanvasTkAgg(fig, master=window)
    canvas_widget = canvas.get_tk_widget()
    canvas_widget.grid(row=0, column=3, rowspan=3)

    def update_map():
        # Update the robot position on the map
        robot_marker.set_xdata(controller.robot_position[0])
        robot_marker.set_ydata(controller.robot_position[1])
        canvas.draw()
        window.after(100, update_map)

    # Start updating the map
    update_map()

    # Bind the stop event to key release
    window.bind("<KeyRelease>", on_stop)

    # Run the GUI event loop
    window.mainloop()

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    create_gui(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
