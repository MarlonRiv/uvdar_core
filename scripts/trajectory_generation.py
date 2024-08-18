import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os



def generate_trajectory(waypoints, hover_times, T=0.2):
    trajectory = []

    # Add the first waypoint
    x0, y0, z0, h0 = waypoints[0]
    trajectory.append((x0, y0, z0, h0))
    
    for i in range(len(waypoints) - 1):
        x0, y0, z0, h0 = waypoints[i]
        x1, y1, z1, h1 = waypoints[i+1]
        hover_time = hover_times[i]
        
        # Calculate number of samples for the hover period
        num_hover_samples = int(hover_time / T)
        trajectory.extend([(x0, y0, z0, h0)] * num_hover_samples)
        
        # Interpolate waypoints
        distance = np.sqrt((x1 - x0)**2 + (y1 - y0)**2 + (z1 - z0)**2)
        num_travel_samples = int(np.ceil(distance / T))
        if num_travel_samples > 0:
            x = np.linspace(x0, x1, num_travel_samples + 1)
            y = np.linspace(y0, y1, num_travel_samples + 1)
            z = np.linspace(z0, z1, num_travel_samples + 1)
            heading = np.linspace(h0, h1, num_travel_samples + 1)
            trajectory.extend([(xi, yi, zi, hi) for xi, yi, zi, hi in zip(x[1:], y[1:], z[1:], heading[1:])])
    
    # Add hover time at the last waypoint
    x_last, y_last, z_last, h_last = waypoints[-1]
    num_final_hover_samples = int(hover_times[-1] / T)
    trajectory.extend([(x_last, y_last, z_last, h_last)] * num_final_hover_samples)
    
    return trajectory


def plot_trajectory(trajectory):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    x, y, z, _ = zip(*trajectory) 

    ax.scatter(x[0], y[0], z[0], color='red', s=100)  

    ax.plot(x, y, z, marker='o', linestyle='-')
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Z Position')
    plt.title('3D Trajectory Visualization')
    plt.show()


def save_trajectory(trajectory, filename):
    with open(filename, 'w') as file:
        for point in trajectory:
            line = ','.join(map(str, point)) + '\n'
            file.write(line)


if __name__ == '__main__':

    waypoints = [
       (-55,15,5,0.5),
       (-55,20,5,0.5),
       (-55,25,5,0.5),
       (-55,30,5,0.5),
       (-55,35,5,0.5),
       (-55,40,5,0.5),
       (-55,40,2,0.5),
   ]


    #0 hover time for each waypoint
    # time = 0
    hover_times = [30,30,30,30,30,30,5]
    # hover_times = [time] * len(waypoints)
    trajectory = generate_trajectory(waypoints, hover_times, T=0.2)
    plot_trajectory(trajectory)
    file_name = 'exp2_traj_tx_h0.5.txt'    
    file_location = os.path.expanduser('~/catkin_ws/src/uvdar_core/config/trajectory/trajectory_files/' + file_name)
    save_trajectory(trajectory, file_location)
