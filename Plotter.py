import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from UtilityFuncs import JOINT_NAMES
import random

class JointValPlotter:
    def __init__(self):
        self.jointvals = []
        self.curr_time = 0
        self.interval = 500
        self.figscale=2.8

        self.fig,self.ax = plt.subplots(5,5, figsize=(3*self.figscale,2*self.figscale))
        self.fig.suptitle('Joint Errors', fontsize=10)
        for i in range(len(JOINT_NAMES)):
            self.ax[i//5, i%5].plot(0,0)
            self.ax[i//5, i%5].set_title(JOINT_NAMES[i],fontsize=8)
            self.ax[i//5, i%5].xaxis.set_tick_params(labelsize=6)
            self.ax[i//5, i%5].yaxis.set_tick_params(labelsize=6)
        
        self.fig.tight_layout()

    def update(self, joint_errors):
        """
        Doesn't really work for now.
        """
        self.curr_time += 1
        for i in range(len(joint_errors)):
            self.ax[i//5, i%5].plot(self.curr_time, joint_errors[i], '-')
        plt.show()

class JointErrorBars:
    def __init__(self):
        self.joint_errors = []
        self.figscale=2.8
        self.fig = plt.figure(figsize=(3*self.figscale,2*self.figscale))
        self.ax = self.fig.add_axes([0,0,1,1])
        self.fig.suptitle('Joint Errors', fontsize=10)
        self.bar = self.ax.bar(JOINT_NAMES,[10]*len(JOINT_NAMES))
        # self.ax.set_xticklabels(JOINT_NAMES, rotation = 90)

    def update(self, i):
        # random.shuffle(currjointerrors)
        self.joint_errors.append(currjointerrors)
        plt.cla()
        self.bar = self.ax.bar(JOINT_NAMES, self.joint_errors[-1])
        # plt.show()
        print(self.joint_errors[-1])
        return self.bar

if __name__ == "__main__":
    # p1 = JointValPlotter()
    # p1.update([1.0]*len(JOINT_NAMES))
    # p1.update([0.5]*len(JOINT_NAMES))
    # p1.update([1.0]*len(JOINT_NAMES))
    # p1.update([0.5]*len(JOINT_NAMES))
    # p1.update([1.0]*len(JOINT_NAMES))
    # p1.update([0.5]*len(JOINT_NAMES))

    p2 = JointErrorBars()
    currjointerrors=[random.randint(0,10) for i in range(len(JOINT_NAMES))]
    anim = FuncAnimation(plt.gcf(), p2.update, interval=100, blit=True)
    plt.show()