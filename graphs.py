import csv
import rospkg
import matplotlib.pyplot as plt

# Load data from CSV file
def load_data(file_path):

    with open(file_path, 'r') as f:
        reader = csv.reader(f)
        data = [list(map(float, row)) for row in reader]
    return zip(*data)  # Transpose data

# Plot data
def plot_data(x, y, theta):
    plt.figure(figsize=(10, 6))
    plt.plot(x, label='X Pose')
    plt.plot(y, label='Y Pose')
    plt.plot(theta, label='Theta')
    plt.xlabel('Time step')
    plt.ylabel('Value')
    plt.title('Pose Data')
    plt.legend()
    plt.show()
    
rospack = rospkg.RosPack()
path = rospack.get_path('omni_control')
file_path = path + '/pose_list_control.csv'

# Load and plot data
x, y, theta = load_data(file_path)
print(x, y, theta)
plot_data(x, y, theta)

