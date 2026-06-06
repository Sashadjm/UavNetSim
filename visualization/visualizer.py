import os
from matplotlib.markers import CARETUP
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
from matplotlib.widgets import Slider, Button, TextBox
from utils import config
import io
from entities.drone import Drone
from entities.user import User
from entities.antenna import Antenna
import matplotlib.patheffects as path_effects

# Add 3D arrow class definition that handles arrows in 3D view
class Arrow3D(FancyArrowPatch):
    """
    Class for drawing arrows in 3D view
    """
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def do_3d_projection(self, renderer=None):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        # Calculate average z value as depth
        avg_z = np.mean(zs)
        return avg_z

    def draw(self, renderer):
        FancyArrowPatch.draw(self, renderer)

class SimulationVisualizer:
    """
    Visualize UAV network simulation process, including movement trajectories and communication status
    """

    def __init__(self, simulator, output_dir="vis_results", vis_frame_interval=50000):
        """
        Initialize visualizer

        Parameters:
            simulator: simulator instance
            output_dir: output directory
            vis_frame_interval: interval for visualization frames (microseconds)
        """
        self.simulator = simulator
        self.output_dir = output_dir

        # Store vis_frame_interval in microseconds
        self.vis_frame_interval = vis_frame_interval

        # Create output directory
        os.makedirs(output_dir, exist_ok=True)

        # Initialize data storage structures
        self.node_positions = {i: [] for i in range(self.simulator.n_nodes)}

        self.user_positions = {i: [] for i in range(self.simulator.n_users)}
        self.timestamps = []

        # Comm events tracking
        self.comm_events = []  # Store tuples (src_id, dst_id, packet_id, packet_type, timestamp)

        # Assign a fixed color to each UAV
        self.colors = plt.cm.tab10(np.linspace(0, 1, self.simulator.n_nodes))


        # Color mapping for communication types
        self.comm_colors = {
            "DATA": "blue",
            "ACK": "green",
            "HELLO": "orange"
        }

        # Setup communication tracking
        self._setup_communication_tracking()

        # Reference for interactive elements
        self.interactive_fig = None
        self.interactive_slider = None
        self.frame_times = []

    def _setup_communication_tracking(self):
        """Setup tracking for communication events"""
        # Save the original unicast_put method
        original_unicast_put = self.simulator.channel.unicast_put

        # Rewrite unicast_put method to track communications
        def tracked_unicast_put(message, dst_node_id):
            # Call the original method
            result = original_unicast_put(message, dst_node_id)

            # Record communication event
            packet, _, src_node_id, _, _ = message

            # Add packet type differentiation
            packet_id = packet.packet_id

            # Identify packet type based on ID range
            if packet_id >= 20000:
                packet_type = "ACK"
            elif packet_id >= 10000:
                packet_type = "HELLO"
            else:
                packet_type = "DATA"

            self.track_communication(src_node_id, dst_node_id, packet_id, packet_type)

            return result

        # Replace the method
        self.simulator.channel.unicast_put = tracked_unicast_put

    def track_node_positions(self):
        """
        Record current node positions
        """
        current_time = self.simulator.env.now / 1e6  # Convert to seconds
        self.timestamps.append(current_time)

        for i, node in enumerate(self.simulator.network_nodes):
            position = node.coords  # This already contains (x, y, z) coordinates
            self.node_positions[i].append(position)

        for i, user in enumerate(self.simulator.users):
            position = user.coords  # This already contains (x, y, z) coordinates
            self.user_positions[i].append(position)

    def track_communication(self, src_id, dst_id, packet_id, packet_type="DATA"):
        """
        Record communication event
        """
        current_time = self.simulator.env.now / 1e6  # Convert to seconds
        # Record complete communication event information
        self.comm_events.append((src_id, dst_id, packet_id, packet_type, current_time))

    def _draw_visualization_frame(self, fig, current_time):
        """
        Draw visualization elements on two side-by-side axes

        Parameters:
            fig: matplotlib figure to draw on
            current_time: current simulation time (seconds)
        """
        fig.suptitle(f"UAV Network Simulation at t={int(current_time*1e6)}μs", fontsize=14)

        # Create left and right subplots for DATA and ACK only
        ax_data = fig.add_subplot(121, projection='3d')
        ax_ack = fig.add_subplot(122, projection='3d')

        # Set titles for subplots
        ax_data.set_title("DATA Packets")
        ax_ack.set_title("ACK Packets")

        # Set axis labels and limits for both subplots
        for ax in [ax_data, ax_ack]:
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            ax.set_xlim(0, config.MAP_LENGTH)
            ax.set_ylim(0, config.MAP_WIDTH)
            ax.set_zlim(0, config.MAP_HEIGHT)
            ax.grid(True)

        # Get node positions at current time
        node_positions = self._get_node_positions(current_time)

        # Draw nodes on both subplots
        for ax in [ax_data, ax_ack]:
            self._draw_nodes(ax, node_positions)

        # Draw communication links
        display_window = self.vis_frame_interval / 1e6  # Convert to seconds
        recent_comms = [e for e in self.comm_events
                      if current_time - display_window <= e[4] <= current_time]

        # Get only the latest communication events for each src-dst pair
        latest_data_comms = self._get_latest_comms(recent_comms, "DATA")
        latest_ack_comms = self._get_latest_comms(recent_comms, "ACK")

        # Draw DATA packet links on left subplot
        self._draw_data_links(ax_data, latest_data_comms, node_positions)

        # Draw ACK packet links on right subplot
        self._draw_ack_links(ax_ack, latest_ack_comms, node_positions)

        # Draw obstacles
        self._draw_obstacles(ax_data)
        self._draw_obstacles(ax_ack)

        # Add legends
        data_legend = [Line2D([0], [0], color=self.comm_colors["DATA"], lw=2, label="DATA Packets")]
        ax_data.legend(handles=data_legend, loc='upper right')

        ack_legend = [Line2D([0], [0], color=self.comm_colors["ACK"], lw=2, label="ACK Packets")]
        ax_ack.legend(handles=ack_legend, loc='upper right')

    def _get_latest_comms(self, comms, packet_type):
        """
        Get only the latest communication for each src-dst pair

        Parameters:
            comms: List of communication events
            packet_type: Type of packet (DATA, ACK, HELLO)

        Returns:
            List of latest communication events for each src-dst pair
        """
        # Filter by packet type
        type_comms = [e for e in comms if e[3] == packet_type]

        # Dictionary to store latest comm for each src-dst pair
        latest_comms_dict = {}

        # For each src-dst pair, keep only the comm with the latest timestamp
        for comm in type_comms:
            src_id, dst_id = comm[0], comm[1]
            pair_key = (src_id, dst_id)

            # If this is the first comm for this pair, or has a later timestamp
            if pair_key not in latest_comms_dict or comm[4] > latest_comms_dict[pair_key][4]:
                latest_comms_dict[pair_key] = comm

        # Return the values (latest comms)
        return list(latest_comms_dict.values())

    def create_animations(self):
        """Create GIF animation of the simulation"""
        import io

        if not self.timestamps:
            print("No timestamps available for animation")
            return

        try:
            print("Creating animation GIF...")
            animation_frames = []

            # Calculate frames based on vis_frame_interval
            min_time = min(self.timestamps)
            max_time = max(self.timestamps)
            frame_interval_sec = self.vis_frame_interval / 1e6  # Convert microseconds to seconds

            # Create frame times at regular intervals based on vis_frame_interval
            self.frame_times = []
            current_time = min_time
            while current_time <= max_time:
                self.frame_times.append(current_time)
                current_time += frame_interval_sec

            n_frames = len(self.frame_times)
            print(f"Generating {n_frames} frames with interval of {frame_interval_sec} seconds")

            for i, time_point in enumerate(self.frame_times):
                print(f"Generating frame {i+1}/{n_frames}", end="\r")

                # Create a new figure for this frame
                fig = plt.figure(figsize=(18, 6))

                # Draw visualization elements
                self._draw_visualization_frame(fig, time_point)

                # Save the figure to a BytesIO buffer
                buf = io.BytesIO()
                plt.savefig(buf, format='png', dpi=100)
                plt.close(fig)

                # Reset buffer position and open image
                buf.seek(0)
                img = Image.open(buf)
                # Convert to RGB mode to ensure compatibility
                img = img.convert('RGB')
                # Create a copy of the image to ensure it's fully loaded
                img_copy = img.copy()
                animation_frames.append(img_copy)
                buf.close()

            print("\nSaving animation...")

            # Save the animation
            animation_file = os.path.join(self.output_dir, "uav_network_simulation.gif")
            if animation_frames:
                # Save with explicit parameters
                animation_frames[0].save(
                    animation_file,
                    format='GIF',
                    save_all=True,
                    append_images=animation_frames[1:],
                    duration=50,  # ms per frame
                    loop=1,  # Loop indefinitely
                    optimize=True,
                    quality=70,    # Reduce quality slightly (0-100)
                    # Reduce colors if needed
                    colors=128     # Maximum number of colors
                )
                print(f"Animation saved to {animation_file}")
            else:
                print("No frames were generated for the animation")

        except Exception as e:
            print(f"Error creating animation: {e}")
            print("Continuing with interactive visualization...")

    def run_visualization(self):
        """
        Run visualization process
        """
        # Use vis_frame_interval directly (it's already in microseconds)
        tracking_interval_us = self.vis_frame_interval

        # Start tracking node positions
        def track_positions():
            while True:
                self.track_node_positions()
                yield self.simulator.env.timeout(tracking_interval_us)

        # Register tracking process
        self.simulator.env.process(track_positions())

    def finalize(self):
        """
        Finalize visualization, generate frames and animations
        """
        print("Finalizing visualization...")

        # Create animation
        self.create_animations()

        # Create interactive visualization
        self.create_interactive_visualization()

        print("Visualization complete. Output saved to:", self.output_dir)

    def create_interactive_visualization(self):
        """Create an interactive visualization with a slider for time navigation"""
        if not self.timestamps:
            print("No timestamps available for interactive visualization")
            return

        print("Creating interactive visualization...")

        # Make sure frame_times is populated
        if not self.frame_times:
            min_time = min(self.timestamps)
            max_time = max(self.timestamps)
            frame_interval_sec = self.vis_frame_interval / 1e6

            current_time = min_time
            while current_time <= max_time:
                self.frame_times.append(current_time)
                current_time += frame_interval_sec

        # Convert frame times to microseconds for the slider
        frame_times_us = [t * 1e6 for t in self.frame_times]

        # Create figure with fixed subplots - this is key to solving the error
        fig = plt.figure(figsize=(15, 7))
        plt.subplots_adjust(bottom=0.15)  # Make room for controls

        # Create the subplots once and keep them
        gs = fig.add_gridspec(1, 2, hspace=0, wspace=0.2)
        ax_data = fig.add_subplot(gs[0, 0], projection='3d')
        ax_ack = fig.add_subplot(gs[0, 1], projection='3d')

        # Add slider axes
        slider_ax = plt.axes([0.2, 0.05, 0.65, 0.03])
        time_slider = Slider(
            slider_ax, 'Time (μs)',
            min(frame_times_us), max(frame_times_us),
            valinit=frame_times_us[0],
            valstep=frame_times_us  # Discrete steps based on frame times
        )

        # Add text box and button for direct time input
        text_ax = plt.axes([0.2, 0.01, 0.2, 0.03])
        time_text = TextBox(text_ax, 'Go to time (μs): ', initial='')

        button_ax = plt.axes([0.45, 0.01, 0.1, 0.03])
        goto_button = Button(button_ax, 'Go')

        def update_plot(current_time):
            # Clear existing content on axes
            ax_data.clear()
            ax_ack.clear()

            # Set titles
            ax_data.set_title("DATA Packets")
            ax_ack.set_title("ACK Packets")

            # Update main figure title
            fig.suptitle(f"UAV Network Simulation at t={int(current_time*1e6)}μs", fontsize=14)

            # Set axis properties for both subplots
            for ax in [ax_data, ax_ack]:
                ax.set_xlabel('X (m)')
                ax.set_ylabel('Y (m)')
                ax.set_zlabel('Z (m)')
                ax.set_xlim(0, config.MAP_LENGTH)
                ax.set_ylim(0, config.MAP_WIDTH)
                ax.set_zlim(0, config.MAP_HEIGHT)
                ax.grid(True)

            # Get node and user positions
            node_positions = self._get_node_positions(current_time)
            # Draw nodes on both subplots
            self._draw_nodes(ax_data, node_positions)
            self._draw_nodes(ax_ack, node_positions)

            # Draw obstacles on both subplots
            self._draw_obstacles(ax_data)
            self._draw_obstacles(ax_ack)

            # Get recent communications
            display_window = self.vis_frame_interval / 1e6
            recent_comms = [e for e in self.comm_events
                            if current_time - display_window <= e[4] <= current_time]

            # Filter by packet type and get latest only
            latest_data_comms = self._get_latest_comms(recent_comms, "DATA")
            latest_ack_comms = self._get_latest_comms(recent_comms, "ACK")

            # Draw communication links
            self._draw_data_links(ax_data, latest_data_comms, node_positions)
            self._draw_ack_links(ax_ack, latest_ack_comms, node_positions)

            # Add legends
            data_legend = [Line2D([0], [0], color=self.comm_colors["DATA"], lw=2, label="DATA Packets")]
            ax_data.legend(handles=data_legend, loc='upper right')

            ack_legend = [Line2D([0], [0], color=self.comm_colors["ACK"], lw=2, label="ACK Packets")]
            ax_ack.legend(handles=ack_legend, loc='upper right')

        def update(val):
            try:
                # Get current time from slider
                current_time_us = time_slider.val
                current_time = current_time_us / 1e6  # Convert to seconds

                # Update plot with new time
                update_plot(current_time)

                # Redraw
                fig.canvas.draw_idle()
            except Exception as e:
                print(f"Error updating plot: {e}")

        def goto_time(event):
            try:
                # Get time from text box
                time_us = float(time_text.text)

                # Find the closest frame time
                closest_time_us = min(frame_times_us, key=lambda x: abs(x - time_us))

                # Update slider to trigger update
                time_slider.set_val(closest_time_us)

                # Update textbox to show actual time used
                time_text.set_val(str(int(closest_time_us)))
            except ValueError:
                print("Invalid time format. Please enter a number.")
            except Exception as e:
                print(f"Error going to time: {e}")

        # Connect the update function to the slider
        time_slider.on_changed(update)

        # Connect the goto function to the button
        goto_button.on_clicked(goto_time)

        # Initial plot
        update_plot(self.frame_times[0])

        # Save reference to interactive elements
        self.interactive_fig = fig
        self.interactive_slider = time_slider

        # Show the interactive visualization
        plt.show()

        print("Interactive visualization created. Close the plot window to continue.")

    def _get_node_positions(self, current_time):
        """Get node positions at a specific time"""
        node_positions = {}
        for node_id in range(len(self.node_positions)):
            positions = self.node_positions[node_id]
            timestamps = self.timestamps

            if positions and timestamps:
                # Find closest timestamp
                closest_idx = min(range(len(timestamps)),
                               key=lambda i: abs(timestamps[i] - current_time))

                # Get position at closest timestamp
                if 0 <= closest_idx < len(positions):
                    node_positions[node_id] = positions[closest_idx]
        return node_positions

    def _draw_nodes(self, ax, node_positions):
        """Draw nodes on the given axis with embedded ID numbers"""
        for node_id, position in node_positions.items():
            # couleur drone
            node = self.simulator.network_nodes[node_id]
            if isinstance(node, Drone):
                color = "#9a0000"
                marker = "1"
            # couleur user
            elif isinstance (node, User):
                color = "#1b4d00"
                marker = "o"
            # couleur antenne
            elif isinstance (node, Antenna):
                color = "#9014af"
                marker = CARETUP

            # Use smaller marker size for node representation
            ax.scatter(position[0], position[1], position[2],
                    marker=marker, color=color, s=150, alpha=0.7, edgecolors='black')

            # Add ID text with outline for better visibility
            # Set high zorder to ensure text appears above other elements
            text = ax.text(position[0], position[1], position[2],
                     f"{node_id}", ha='center', va='center',
                     color='white', fontweight='bold', fontsize=10,
                     path_effects=[path_effects.withStroke(linewidth=2, foreground='black')],
                     zorder=100)  # Ensure text is displayed on top layer

    def _draw_data_links(self, ax, data_comms, node_positions):
        """Draw DATA packet links on the given axis with smaller packet ID boxes"""
        for src_id, dst_id, packet_id, _, _ in data_comms:
            if src_id in node_positions and dst_id in node_positions:
                start_pos = node_positions[src_id]
                end_pos = node_positions[dst_id]

                # Draw an arrow for DATA packet
                arrow = Arrow3D([start_pos[0], end_pos[0]],
                              [start_pos[1], end_pos[1]],
                              [start_pos[2], end_pos[2]],
                              mutation_scale=15,
                              lw=2,
                              arrowstyle="-|>",
                              color=self.comm_colors["DATA"])

                ax.add_artist(arrow)

                # Add more visible packet ID at midpoint
                mid_x, mid_y, mid_z = [(start_pos[i] + end_pos[i]) / 2 for i in range(3)]

                # Draw a smaller, more compact background for the packet ID
                ax.text(mid_x, mid_y, mid_z, str(packet_id),
                      ha='center', va='center', fontsize=7, fontweight='bold',
                      bbox=dict(boxstyle="round,pad=0.2", facecolor='lightblue',
                                alpha=0.8, edgecolor=self.comm_colors["DATA"], linewidth=1.5),
                      zorder=99)  # Display above other elements but below node IDs

    def _draw_ack_links(self, ax, ack_comms, node_positions):
        """Draw ACK packet links on the given axis with smaller packet ID boxes"""
        for src_id, dst_id, packet_id, _, _ in ack_comms:
            if src_id in node_positions and dst_id in node_positions:
                start_pos = node_positions[src_id]
                end_pos = node_positions[dst_id]

                # Draw a straight line for ACK packet
                ax.plot([start_pos[0], end_pos[0]],
                       [start_pos[1], end_pos[1]],
                       [start_pos[2], end_pos[2]],
                       color=self.comm_colors["ACK"],
                       linewidth=2)

                # Add more visible packet ID at midpoint
                mid_x, mid_y, mid_z = [(start_pos[i] + end_pos[i]) / 2 for i in range(3)]

                # Draw a smaller, more compact background for the ACK packet ID
                ax.text(mid_x, mid_y, mid_z, str(packet_id),
                       ha='center', va='center', fontsize=7, fontweight='bold',
                       bbox=dict(boxstyle="round,pad=0.2", facecolor='lightgreen',
                                alpha=0.8, edgecolor=self.comm_colors["ACK"], linewidth=1.5),
                       zorder=99)  # Display above other elements but below node IDs

    def _draw_obstacles(self, ax):
        """Draw transparent 3D voxel obstacles and set the physical aspect ratio."""
        filled = self.simulator.grid > 0
        res = config.GRID_RESOLUTION

        # Generate physical mesh coordinates
        x_edges = np.linspace(0, config.MAP_LENGTH, filled.shape[0] + 1)
        y_edges = np.linspace(0, config.MAP_WIDTH, filled.shape[1] + 1)
        z_edges = np.linspace(0, config.MAP_HEIGHT, filled.shape[2] + 1)
        X, Y, Z = np.meshgrid(x_edges, y_edges, z_edges, indexing='ij')

        # Render voxels
        ax.voxels(X, Y, Z, filled, facecolors=[0.0, 0.6, 1.0, 0.2])

        # Maintain physical proportions (ensures 600x600x100 space looks correct)
        ax.set_box_aspect([config.MAP_LENGTH, config.MAP_WIDTH, config.MAP_HEIGHT])
