import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn  # Import the OverrideRCIn message
import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import os
import subprocess
from mavros_msgs.srv import CommandBool


script_directory = "/home/robot/mavros_ws/src/mavros/mavros/launch"
ros_setup_script = "/opt/ros/humble/setup.bash"  # Replace <ros2_distro> with your ROS 2 distribution.

try:
    # Open a new terminal, navigate to the directory, and run the ROS 2 launch command
    subprocess.run([
        "gnome-terminal", "--", "bash", "-c",
        f"cd {script_directory} && source {ros_setup_script} && ros2 launch mavros apm.launch; exec bash"
    ])
    print("Skrypt został uruchomiony w nowym terminalu z poprawnego katalogu!")
except FileNotFoundError as e:
    print(f"Nie znaleziono pliku: {e}")
except subprocess.CalledProcessError as e:
    print(f"Wystąpił błąd podczas uruchamiania skryptu: {e}")


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        # Create a publisher for /mavros/rc/override topic
        self.publisher_ = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.override_rc_in = OverrideRCIn()
        self.override_rc_in.channels = [0] * 18  # Initialize all channels to zero

        # Initialize robot_position to [0.0, 0.0]
        self.robot_position = [0.0, 0.0]  # Fix: Initialize robot_position

    def publish_manual_override(self, channels):
        """Publishes an OverrideRCIn message with manually set channel values.

        Args:
            channels (list): A list of 18 integer values representing channel inputs.
        """
        if len(channels) != 18:
            self.get_logger().warn("Channels list must contain exactly 18 values.")
            return

        self.override_rc_in.channels = channels
        self.publisher_.publish(self.override_rc_in)

    # Example usage methods for different motions:
    def move_forward(self):
        self.publish_manual_override([2000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

    def move_backward(self):
        self.publish_manual_override([1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

    def turn_left(self):
        self.publish_manual_override([0, 2000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

    def turn_right(self):
        self.publish_manual_override([0, 0, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

    def stop_robot(self):
        self.publish_manual_override([0] * 18)

    def arm_robot(self):
        # instruction
        # Open new terminal to call the arming service command
        try:
            subprocess.run([
                "gnome-terminal", "--", "bash", "-c",
                "ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool '{value: true}'; exec bash"
            ])
        except Exception as e:
            print(f"Failed to execute arming command: {e}")


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

    def on_arm():
        controller.arm_robot()

    # Create the main window
    window = tk.Tk()
    window.title("Robot Teleop GUI with Map")

    # Create buttons
    forward_button = tk.Button(window, text="↑", command=on_forward, width=10, height=2)
    backward_button = tk.Button(window, text="↓", command=on_backward, width=10, height=2)
    left_button = tk.Button(window, text="←", command=on_left, width=10, height=2)
    right_button = tk.Button(window, text="→", command=on_right, width=10, height=2)
    arm_button = tk.Button(window, text="Arm Robot", command=on_arm, width=10, height=2)

    # Layout the buttons in a grid
    forward_button.grid(row=0, column=1)
    left_button.grid(row=1, column=0)
    right_button.grid(row=1, column=2)
    backward_button.grid(row=2, column=1)
    arm_button.grid(row=3, column=1)

    # Create a Figure for the map
    fig = Figure(figsize=(5, 5))
    ax = fig.add_subplot(111)
    ax.set_xlim(-10, 10)  # Set the map limits
    ax.set_ylim(-10, 10)
    robot_marker, = ax.plot(0, 0, 'ro')  # Robot position marker

    # Embed the Figure in the tkinter window
    canvas = FigureCanvasTkAgg(fig, master=window)
    canvas_widget = canvas.get_tk_widget()
    canvas_widget.grid(row=0, column=3, rowspan=4)

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
