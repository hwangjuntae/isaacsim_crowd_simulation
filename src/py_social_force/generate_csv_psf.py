from pathlib import Path
import numpy as np
import random
import pysocialforce as psf
import matplotlib.pyplot as plt
import pandas as pd  # For saving CSV files

# Seed for reproducibility (optional)
# random.seed(42)
# np.random.seed(42)

# Create directories for images and CSV files
Path("images").mkdir(exist_ok=True)
Path("csv").mkdir(exist_ok=True)

def random_spawn_x(left_side=True):
    """Generate a random x-coordinate for spawning on the left or right side."""
    if left_side:
        return random.uniform(-30, 0)  # Left side (-30 to 0)
    else:
        return random.uniform(0, 30)   # Right side (0 to 30)

def random_goal_x(spawn_x):
    """Set the goal x-coordinate based on the spawn position."""
    if spawn_x < 0:
        return random.uniform(25, 30)   # Goal on the right
    else:
        return random.uniform(-30, -25) # Goal on the left

def random_spawn_y():
    """Generate a random y-coordinate within the corridor (-4.5 to 4.5)."""
    return random.uniform(-4.5, 4.5)  # Within the corridor

# 사람 수 설정
def run_simulation(run_number, total_agents=20, num_steps=300):
    """
    Run a single simulation and save the results.

    Parameters:
    - run_number: int, the current simulation run number
    - total_agents: int, number of agents in the simulation
    - num_steps: int, number of simulation steps
    """
    # Define base name with run number
    base_name = f"crowd_coordinates{run_number}"
    
    initial_state = []
    groups = []
    group_colors = []
    agent_labels = []
    group_start_index = 0
    remaining_agents = total_agents

    # Generate initial state for agents
    while remaining_agents > 0:
        group_size = min(random.randint(1, 5), remaining_agents)
        remaining_agents -= group_size

        left_side = random.choice([True, False])

        center_x = random_spawn_x(left_side=left_side)
        center_y = random_spawn_y()

        goal_x = random_goal_x(center_x)
        goal_y = random_spawn_y()

        group_speed = random.uniform(0.8, 1.2)  # Speed between 0.8 and 1.2 m/s
        angle = np.arctan2(goal_y - center_y, goal_x - center_x)  # Direction angle

        group = []
        for _ in range(group_size):
            pos_x = random.uniform(center_x - 0.5, center_x + 0.5)
            pos_y = random.uniform(center_y - 0.5, center_y + 0.5)

            speed_variation = random.uniform(-0.05, 0.05)
            agent_speed = group_speed + speed_variation

            vel_x = agent_speed * np.cos(angle)
            vel_y = agent_speed * np.sin(angle)

            agent_state = [
                pos_x,   # x position
                pos_y,   # y position
                vel_x,   # x velocity
                vel_y,   # y velocity
                goal_x,  # x goal
                goal_y   # y goal
            ]
            group.append(agent_state)

        initial_state.extend(group)
        groups.append(list(range(group_start_index, group_start_index + group_size)))
        agent_labels.extend(range(group_start_index, group_start_index + group_size))

        # Assign color based on spawn side
        if left_side:
            group_colors.extend(["red"] * group_size)
        else:
            group_colors.extend(["blue"] * group_size)

        group_start_index += group_size

    initial_state = np.array(initial_state)

    # Define corridor walls as obstacles
    obs = [
        [-30, 30, 5, 5],    # Top boundary
        [-30, 30, -5, -5],  # Bottom boundary
    ]

    # Initialize the simulator
    config_path = Path(__file__).resolve().parent.joinpath("psf_config.toml")
    s = psf.Simulator(
        initial_state,
        groups=groups,
        obstacles=obs,
        config_file=config_path,
    )

    # Print available configuration attributes (optional)
    # print("Available configuration attributes:")
    # print(dir(s.config))
    # print("\nConfiguration details:")
    # print(s.config)

    num_agents = len(initial_state)
    
    # List to store simulation data
    data = []

    # Run the simulation
    for step in range(num_steps):
        s.step()

        # Calculate current simulation time
        try:
            time = step * s.config.simulation.dt
        except AttributeError:
            try:
                time = step * s.config.dt
            except AttributeError:
                time = step * 0.4  # Default value if dt is not found

        # Get current agent states
        positions = s.peds.state.copy()

        for i in range(num_agents):
            agent_id = agent_labels[i]
            x = positions[i, 0]
            y = positions[i, 1]
            vx = positions[i, 2]
            vy = positions[i, 3]
            goal_x = positions[i, 4]
            goal_y = positions[i, 5]

            data.append({
                'time': time,
                'agent_id': agent_id,
                'x': x,
                'y': y,
                'vx': vx,
                'vy': vy,
                'goal_x': goal_x,
                'goal_y': goal_y
            })

    # Convert data to DataFrame
    df = pd.DataFrame(data)

    # Save CSV
    output_csv_path = Path("csv") / f"{base_name}.csv"
    df.to_csv(output_csv_path, index=False)
    print(f"[Run {run_number}] Simulation data saved to '{output_csv_path}'.")

    # Visualization and GIF creation
    output_gif_path = Path("images") / base_name  # '.gif' will be appended by the visualizer

    # Debugging output (optional)
    # print(f"output_gif_path type: {type(output_gif_path)}, value: {output_gif_path}")

    with psf.plot.SceneVisualizer(s, str(output_gif_path)) as sv:
        sv.agent_colors = group_colors  # Set agent colors

        plt.xlim(-30, 30)  # Set x-axis limits
        plt.ylim(-10, 10)  # Set y-axis limits
        sv.animate()
    print(f"[Run {run_number}] Simulation GIF saved to '{output_gif_path}.gif'.")

if __name__ == "__main__":
    total_runs = 3  # Number of simulations to run

    for run in range(1, total_runs + 1):
        print(f"Starting simulation run {run}/{total_runs}...")
        run_simulation(run_number=run)
        print(f"Completed simulation run {run}/{total_runs}.\n")
    
    print("All simulations completed successfully.")
