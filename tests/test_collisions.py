import pybullet as p
import pybullet_data
import time
import numpy as np
import os 
from python_robotics_middleware import Obstacle, Sphere, Cube

def are_tuples_close(tuple1, tuple2, tolerance=1e-3):
    """
    Compare two tuples (including nested ones) element-wise within a given tolerance.

    Args:
        tuple1: First tuple of values (can be nested).
        tuple2: Second tuple of values (can be nested).
        tolerance: Maximum allowed difference for each element.

    Returns:
        True if all elements are within the tolerance, False otherwise.
    """
    if type(tuple1) != type(tuple2):
        return False

    if isinstance(tuple1, (tuple, list)) and isinstance(tuple2, (tuple, list)):
        if len(tuple1) != len(tuple2):
            return False
        return all(are_tuples_close(a, b, tolerance) for a, b in zip(tuple1, tuple2))

    return abs(tuple1 - tuple2) <= tolerance

def visualize_collision_with_sphere(data, urdf, visualize=True, delay=True):
    """
    Visualize pre-computed joint configurations in PyBullet with collision detection.
    
    Args:
        data: List of dictionaries containing pre-computed joint angles for robot poses.
        urdf: Path to the robot URDF file.
        n: Number of poses to visualize.
        visualize: Whether to visualize the robot in PyBullet.
        delay: Whether to add a delay between visualizing poses.
    """
    if visualize:
        # Initialize PyBullet in GUI mode
        p.connect(p.GUI)
        p.setRealTimeSimulation(False)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Set up camera for better visualization
        yaw = 90
        pitch = -35
        dist = 2
        target = np.array([0, 0, 0])
        p.resetDebugVisualizerCamera(dist, yaw, pitch, target)

        # Load ground plane
        plane_id = p.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)
        p.changeVisualShape(plane_id, -1, rgbaColor=[0.3, 0.3, 0.3, 1])

        # Load robot model
        robot_id = p.loadURDF(urdf, basePosition=[0, 0, 0], useFixedBase=True)
        
        sphere = Sphere(name="sphere_1",color=[1,0,0,1], position=[0.5,0.5,0.2],mass=0, radius=0.2)

        sphere_id = sphere.generate_pybullet_obstacle()

      

        # Keep the visualization running until manually exited
        cached_contact = None
        while True:
            contact_points_with_obstacle = p.getContactPoints(bodyA=robot_id, bodyB=sphere_id)

            # print(contact_points_with_obstacle)

            if contact_points_with_obstacle:
                for contact in contact_points_with_obstacle:
                    if are_tuples_close(contact[5], cached_contact, tolerance=1e-3):
                        print(f"Contact at position {contact[5]}")
                    cached_contact = contact[5]

            
            p.stepSimulation()
def visualize_collision_with_cube(data, urdf, visualize=True, delay=True):
    """
    Visualize pre-computed joint configurations in PyBullet with collision detection.
    
    Args:
        data: List of dictionaries containing pre-computed joint angles for robot poses.
        urdf: Path to the robot URDF file.
        n: Number of poses to visualize.
        visualize: Whether to visualize the robot in PyBullet.
        delay: Whether to add a delay between visualizing poses.
    """
    if visualize:
        # Initialize PyBullet in GUI mode
        p.connect(p.GUI)
        p.setRealTimeSimulation(False)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Set up camera for better visualization
        yaw = 90
        pitch = -35
        dist = 2
        target = np.array([0, 0, 0])
        p.resetDebugVisualizerCamera(dist, yaw, pitch, target)

        # Load ground plane
        plane_id = p.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)
        p.changeVisualShape(plane_id, -1, rgbaColor=[0.3, 0.3, 0.3, 1])

        # Load robot model
        robot_id = p.loadURDF(urdf, basePosition=[0, 0, 0], useFixedBase=True)
        
        cube = Cube(name="cube_1",color=[1,1,0,0.5], position=[0.5,1.0,0.2],mass=1, side_length=0.2)

        cube_id = cube.generate_pybullet_obstacle()

      

        cached_contact = None
        while True:
            contact_points_with_obstacle = p.getContactPoints(bodyA=robot_id, bodyB=cube_id)

            # print(contact_points_with_obstacle)

            if contact_points_with_obstacle:
                for contact in contact_points_with_obstacle:
                    if are_tuples_close(contact[5], cached_contact, tolerance=1e-3):
                        print(f"Contact at position {contact[5]}")
                    cached_contact = contact[5]

            
            p.stepSimulation()

def visualize_collision_with_loaded_obstacle(data, urdf, obstacle, n=10, visualize=True, delay=False):
    """
    Visualize pre-computed joint configurations in PyBullet with collision detection.

    Args:
        data: List of dictionaries containing pre-computed joint angles for robot poses.
        urdf: Path to the robot URDF file.
        obstacle: An instance of the Obstacle class or derived class (e.g., Sphere, Cube).
        n: Number of poses to visualize.
        visualize: Whether to visualize the robot in PyBullet.
        delay: Whether to add a delay between visualizing poses.
    """
    if visualize:
        # Initialize PyBullet in GUI mode
        p.connect(p.GUI)
        p.setRealTimeSimulation(False)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Set up camera for better visualization
        yaw = 90
        pitch = -35
        dist = 2
        target = np.array([0, 0, 0])
        p.resetDebugVisualizerCamera(dist, yaw, pitch, target)

        # Load ground plane
        plane_id = p.loadURDF("plane.urdf", [0, 0, 0], useFixedBase=True)
        p.changeVisualShape(plane_id, -1, rgbaColor=[0.3, 0.3, 0.3, 1])

        # Load robot model
        robot_id = p.loadURDF(urdf, basePosition=[0, 0, 0], useFixedBase=True)

        # Generate and load the obstacle from the Obstacle object
        obstacle_id = obstacle.generate_pybullet_obstacle()

        # Visualize robot poses and check for collisions
        cached_contact = None
        for _ in range(n):
            # Simulate robot movement (update poses)
            # (For example, you can set joint configurations using `data` if necessary)

            # Check for collisions between the robot and the obstacle
            contact_points_with_obstacle = p.getContactPoints(bodyA=robot_id, bodyB=obstacle_id)

            if contact_points_with_obstacle:
                for contact in contact_points_with_obstacle:
                    if are_tuples_close(contact[5], cached_contact, tolerance=1e-3):
                        print(f"Contact at position {contact[5]}")
                    cached_contact = contact[5]

            # Optionally add a delay between visualizing poses
            if delay:
                time.sleep(0.1)  # Adjust the delay as needed

            p.stepSimulation()
        
        # Disconnect PyBullet when done
        p.disconnect()
if __name__ == "__main__":
    search_path = pybullet_data.getDataPath()

    # Define URDF and dataset
    urdf = "franka/fp3_franka_hand.urdf"
    data = [
        {'joint_angles': [0.0, -0.5, 0.5, -1.0, 0.0, 0.8, 0.0]},  # Example pose 1
        {'joint_angles': [0.2, -0.3, 0.7, -0.9, 0.1, 0.5, 0.0]},  # Example pose 2
        # Add more poses here...
    ]

    sdf_path = "test_sdfs/cinder_block/model.sdf"
    # sdf_path = "test_sdfs/table.sdf"

    # Visualize pre-computed joint configurations
    # visualize_collision_with_sphere(data=data, urdf=urdf)
    visualize_collision_with_cube(data=data, urdf=urdf)
    # visualize_collision_with_loaded_obstacle(data=data, urdf=urdf, sdf_path=sdf_path, n=10)
