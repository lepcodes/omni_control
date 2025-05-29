import csv
import rospkg
import pickle
import numpy as np
import matplotlib.pyplot as plt




def load_data(file_path):
    with open(file_path, 'r') as f:
        reader = csv.reader(f)
        data = [list(map(float, row)) for row in reader]
    return zip(*data)  # Transpose data

def plot_data(x_real, y_real, theta_real, x_desired, y_desired, theta_desired):
    t = np.linspace(0, len(x_real)*0.1, len(x_real))
    plt.figure(figsize=(9, 5))
    plt.plot(t, x_real,  label='x', linewidth=2)
    plt.plot(t, y_real, label='y', linewidth=2)
    plt.plot(t, theta_real, label='Theta', linewidth=2)
    plt.plot(t, x_desired,'--', label='x_d', linewidth=2)
    plt.plot(t, y_desired,'--', label='x_d', linewidth=2)
    plt.plot(t, theta_desired,'--', label='theta_d', linewidth=2)
    plt.title('Robot Trajectory')
    plt.xlabel('Time (s)')
    plt.ylabel('Distance')
    plt.legend()
    plt.grid(True)
    
    plt.show()

# Specify your file paths
rospack = rospkg.RosPack()
path = rospack.get_path('omni_control')
file_path_real = path + '/pose_list_control.csv'
file_path_desired = path + '/pose_d_list_control.csv'

pickle_file_path = path + '/np_array.pkl'
with open(pickle_file_path, 'rb') as f:
    loaded_list_of_np_arrays = pickle.load(f)
    print(loaded_list_of_np_arrays)
    print(len(loaded_list_of_np_arrays))
# Load and plot data
x_real, y_real, theta_real = load_data(file_path_real)
x_desired, y_desired, theta_desired = load_data(file_path_desired)
print(len(x_real))
print(len(np.linspace(0, len(x_real)*0.1, len(x_real))))

plot_data(x_real, y_real, theta_real, x_desired, y_desired, theta_desired)

