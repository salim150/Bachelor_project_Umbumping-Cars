import pandas as pd

import matplotlib.pyplot as plt
import seaborn as sns

# Read the CSV file
data = pd.read_csv('src/seed_simulation/seed_simulation/seed_sim.csv')
# data = data.to_numpy()

class Plotter:
    def __init__(self, data):
        self.data = data

    def plot(self, x, y, hue):
        
        sns.lineplot(data=self.data, x=x, y=y, hue=hue)
        plt.show()


# Plot the quantities as a function of variables and noise
# dwa_data = data.loc[data["Method"] == 'DWA']
# dwa_plotter = Plotter(dwa_data)
# dwa_plotter.plot("Robot Number", 'Collision Number', 'Noise Scaling')

# mpc_data = data.loc[data["Method"] == 'MPC']
# mpc_plotter = Plotter(mpc_data)
# mpc_plotter.plot("Robot Number", 'Path Length', 'Noise Scaling')

# lbp_data = data.loc[data["Method"] == 'LBP']
# lbp_plotter = Plotter(lbp_data)
# lbp_plotter.plot("Robot Number", 'Path Length', 'Noise Scaling')

# cbf_data = data.loc[data["Method"] == 'CBF']
# cbf_plotter = Plotter(cbf_data)
# cbf_plotter.plot("Robot Number", 'Path Length', 'Noise Scaling')

c3bf_data = data.loc[data["Method"] == 'C3BF']
c3bf_plotter = Plotter(c3bf_data)
c3bf_plotter.plot("Robot Number", 'Collision Number', 'Noise Scaling')
