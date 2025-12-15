import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import splprep, splev
from typing import List
from planning.trajectory_recorder import TrajectoryPoint
from utils.logger import setup_logger

logger = setup_logger("TrajectoryVisualizer")

class TrajectoryVisualizer:
    """
    Visualizes drone trajectory in real-time and post-flight with Bezier smoothing.
    """

    def __init__(self, enable_realtime=True, smooth_factor=2.0):
        """
        Initialize trajectory visualizer.

        Args:
            enable_realtime: Enable real-time plotting during flight (default True)
            smooth_factor: Smoothing factor for Bezier curves (default 2.0)
        """
        self.enable_realtime = enable_realtime
        self.smooth_factor = smooth_factor

        self.fig = None
        self.ax_2d = None
        self.ax_3d = None

        if self.enable_realtime:
            self._setup_realtime_plot()

        logger.info(f"TrajectoryVisualizer initialized: realtime={enable_realtime}, smooth_factor={smooth_factor}")

    def _setup_realtime_plot(self) -> None:
        """Setup matplotlib figure for real-time visualization"""
        try:
            plt.ion()  # Interactive mode
            self.fig = plt.figure(figsize=(12, 10))

            # 2D top-down view
            self.ax_2d = self.fig.add_subplot(211)
            self.ax_2d.set_xlabel('X (m)')
            self.ax_2d.set_ylabel('Y (m)')
            self.ax_2d.set_title('Drone Trajectory - Top-Down View')
            self.ax_2d.grid(True)
            self.ax_2d.axis('equal')

            # 3D view
            self.ax_3d = self.fig.add_subplot(212, projection='3d')
            self.ax_3d.set_xlabel('X (m)')
            self.ax_3d.set_ylabel('Y (m)')
            self.ax_3d.set_zlabel('Z (m)')
            self.ax_3d.set_title('Drone Trajectory - 3D View')

            plt.tight_layout()
            logger.info("Real-time visualization setup complete")

        except Exception as e:
            logger.error(f"Error setting up real-time plot: {e}")
            self.enable_realtime = False

    def update_realtime(self, trajectory: List[TrajectoryPoint]) -> None:
        """
        Update real-time visualization (non-blocking).

        Args:
            trajectory: List of trajectory points recorded so far
        """
        if not self.enable_realtime or self.fig is None:
            return

        if len(trajectory) < 2:
            return  # Need at least 2 points to plot

        try:
            # Extract positions
            positions = np.array([p.position for p in trajectory])
            timestamps = np.array([p.timestamp for p in trajectory])

            # 2D Top-down view
            self.ax_2d.clear()
            self.ax_2d.plot(positions[:, 0], positions[:, 1], 'b-', alpha=0.6, linewidth=1.5, label='Path')
            self.ax_2d.scatter(positions[0, 0], positions[0, 1], c='green', s=100, marker='o', label='Start', zorder=5)
            self.ax_2d.scatter(positions[-1, 0], positions[-1, 1], c='red', s=100, marker='o', label='Current', zorder=5)

            self.ax_2d.set_xlabel('X (m)')
            self.ax_2d.set_ylabel('Y (m)')
            self.ax_2d.set_title(f'Trajectory - Top View (t={timestamps[-1]:.1f}s)')
            self.ax_2d.grid(True)
            self.ax_2d.axis('equal')
            self.ax_2d.legend()

            # 3D view
            self.ax_3d.clear()
            self.ax_3d.plot(positions[:, 0], positions[:, 1], positions[:, 2],
                           'b-', alpha=0.6, linewidth=1.5, label='Path')
            self.ax_3d.scatter(positions[0, 0], positions[0, 1], positions[0, 2],
                              c='green', s=100, marker='o', label='Start')
            self.ax_3d.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2],
                              c='red', s=100, marker='o', label='Current')

            self.ax_3d.set_xlabel('X (m)')
            self.ax_3d.set_ylabel('Y (m)')
            self.ax_3d.set_zlabel('Z (m)')
            self.ax_3d.set_title(f'Trajectory - 3D View ({len(trajectory)} points)')
            self.ax_3d.legend()

            plt.pause(0.01)  # Non-blocking update

        except Exception as e:
            logger.error(f"Error updating real-time visualization: {e}")

    def smooth_trajectory_bezier(self, positions: np.ndarray, num_points=500) -> np.ndarray:
        """
        Smooth trajectory using Bezier curve (B-spline) interpolation.

        Args:
            positions: Array of positions (N, 3) where N is number of points
            num_points: Number of points in smoothed trajectory (default 500)

        Returns:
            Smoothed trajectory as numpy array (num_points, 3)
        """
        if len(positions) < 4:
            logger.warning("Need at least 4 points for Bezier smoothing, returning original")
            return positions

        try:
            # Parametric B-spline fitting
            # s=smooth_factor controls smoothing (0=exact fit, higher=smoother)
            # k=3 for cubic splines
            tck, u = splprep([positions[:, 0], positions[:, 1], positions[:, 2]],
                            s=self.smooth_factor, k=3)

            # Evaluate spline at higher resolution
            u_fine = np.linspace(0, 1, num_points)
            x_smooth, y_smooth, z_smooth = splev(u_fine, tck)

            smoothed = np.column_stack([x_smooth, y_smooth, z_smooth])
            logger.debug(f"Trajectory smoothed: {len(positions)} points → {num_points} points")

            return smoothed

        except Exception as e:
            logger.error(f"Error in Bezier smoothing: {e}, returning original trajectory")
            return positions

    def visualize_post_flight(self, trajectory: List[TrajectoryPoint], save_path: str) -> bool:
        """
        Create comprehensive post-flight visualization with Bezier smoothing.

        Args:
            trajectory: Complete trajectory from flight
            save_path: Path to save PNG file

        Returns:
            True if successful, False otherwise
        """
        if len(trajectory) < 2:
            logger.warning("Need at least 2 points for post-flight visualization")
            return False

        try:
            # Extract data
            positions = np.array([p.position for p in trajectory])
            timestamps = np.array([p.timestamp for p in trajectory])
            battery = np.array([p.battery for p in trajectory])
            states = [p.state for p in trajectory]

            # Smooth trajectory
            smoothed_positions = self.smooth_trajectory_bezier(positions)

            # Create 4-panel figure
            fig = plt.figure(figsize=(16, 12))

            # Panel 1: 3D trajectory with Bezier smoothing
            ax1 = fig.add_subplot(221, projection='3d')
            ax1.plot(smoothed_positions[:, 0], smoothed_positions[:, 1], smoothed_positions[:, 2],
                    'b-', linewidth=2, label='Smoothed Path (Bezier)', alpha=0.8)
            scatter = ax1.scatter(positions[:, 0], positions[:, 1], positions[:, 2],
                                 c=timestamps, cmap='viridis', s=10, alpha=0.5, label='Actual Points')
            ax1.scatter(positions[0, 0], positions[0, 1], positions[0, 2],
                       c='green', s=200, marker='*', label='Start', edgecolors='black', linewidths=2)
            ax1.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2],
                       c='red', s=200, marker='*', label='End', edgecolors='black', linewidths=2)

            ax1.set_xlabel('X (m)')
            ax1.set_ylabel('Y (m)')
            ax1.set_zlabel('Z (m)')
            ax1.set_title('3D Flight Path (Bezier Smoothed)')
            ax1.legend()
            plt.colorbar(scatter, ax=ax1, label='Time (s)', shrink=0.6)

            # Panel 2: Top-down view
            ax2 = fig.add_subplot(222)
            ax2.plot(smoothed_positions[:, 0], smoothed_positions[:, 1],
                    'b-', linewidth=2, alpha=0.8, label='Smoothed Path')
            scatter2 = ax2.scatter(positions[:, 0], positions[:, 1],
                                  c=timestamps, cmap='viridis', s=20, alpha=0.6)
            ax2.scatter(positions[0, 0], positions[0, 1],
                       c='green', s=200, marker='*', label='Start', edgecolors='black', linewidths=2)
            ax2.scatter(positions[-1, 0], positions[-1, 1],
                       c='red', s=200, marker='*', label='End', edgecolors='black', linewidths=2)

            ax2.set_xlabel('X (m)')
            ax2.set_ylabel('Y (m)')
            ax2.set_title('Top-Down View')
            ax2.axis('equal')
            ax2.grid(True, alpha=0.3)
            ax2.legend()
            plt.colorbar(scatter2, ax=ax2, label='Time (s)')

            # Panel 3: Altitude profile over time
            ax3 = fig.add_subplot(223)
            ax3.plot(timestamps, positions[:, 2], 'r-', linewidth=2, label='Altitude (Z)')
            ax3.fill_between(timestamps, 0, positions[:, 2], alpha=0.3, color='red')
            ax3.set_xlabel('Time (s)')
            ax3.set_ylabel('Altitude (m)')
            ax3.set_title('Altitude Profile Over Time')
            ax3.grid(True, alpha=0.3)
            ax3.legend()

            # Panel 4: Battery consumption over time
            ax4 = fig.add_subplot(224)
            ax4.plot(timestamps, battery, 'g-', linewidth=2, label='Battery Level')
            ax4.fill_between(timestamps, 0, battery, alpha=0.3, color='green')
            ax4.set_xlabel('Time (s)')
            ax4.set_ylabel('Battery (%)')
            ax4.set_title('Battery Consumption')
            ax4.grid(True, alpha=0.3)
            ax4.legend()

            # Add summary statistics
            duration = timestamps[-1]
            distance = np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1))
            battery_used = battery[0] - battery[-1]

            summary_text = (
                f"Flight Statistics:\n"
                f"Duration: {duration:.1f}s\n"
                f"Distance: {distance:.2f}m\n"
                f"Battery Used: {battery_used:.1f}%\n"
                f"Points Recorded: {len(trajectory)}"
            )
            fig.text(0.02, 0.02, summary_text, fontsize=10, family='monospace',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

            plt.tight_layout(rect=[0, 0.05, 1, 1])  # Make room for summary text
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            logger.info(f"Post-flight visualization saved to {save_path}")

            plt.show()
            return True

        except Exception as e:
            logger.error(f"Error creating post-flight visualization: {e}")
            return False

    def close(self) -> None:
        """Close visualization windows"""
        if self.fig is not None:
            plt.close(self.fig)
            self.fig = None
        logger.info("Visualization windows closed")
