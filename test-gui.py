from tkinter import *
from PIL import ImageTk, Image
import glob
# Might want to look into https://pypi.org/project/pybullet-rendering/

root = Tk()
root.title('Test GUI')

label_1 = Label()
label_1.grid(row=0, column=0, columnspan=3)

