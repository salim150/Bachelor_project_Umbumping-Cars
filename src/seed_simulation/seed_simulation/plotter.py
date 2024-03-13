import pandas as pd

import matplotlib.pyplot as plt
import seaborn as sns

# Read the CSV file
data = pd.read_csv('src/seed_simulation/seed_simulation/seed_sim.csv')
# data = data.to_numpy()
fontsize = 10
plt.rcParams['font.family'] = ['serif']
plt.rcParams['font.serif'] = ['Times New Roman']
plt.rcParams['font.size'] = fontsize

class Plotter:
    def __init__(self, data):
        self.data = data

    def plot(self, x, y, hue, title):
        
        sns.lineplot(data=self.data, x=x, y=y, hue=hue)
        # plt.xlabel("x [m]", fontdict={'size': fontsize, 'family': 'serif'})
        # plt.ylabel("y [m]", fontdict={'size': fontsize, 'family': 'serif'})
        plt.title(title, fontdict={'size': fontsize, 'family': 'serif'})
        plt.show()

quantities = ['Path Length', 'Acceleration Usage', 'Steering Usage', 'Average Speed', 'Avg Computational Time',	'Solver Failure', 'Collision Number']
methods = ['MPC', 'LBP', 'CBF', 'C3BF', 'DWA']

for method in methods:
    for quantity in quantities:
        mpc_data = data.loc[data["Method"] == method]
        mpc_plotter = Plotter(mpc_data)
        mpc_plotter.plot("Robot Number", quantity, 'Noise Scaling', quantity+" vs Robot Number " + '(' + method + ')')


