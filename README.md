# PyBullet-Nao-Motion-Editor
A motion editor for the Nao robot, primarily for RoboCup.

## Setup
1. Clone the repository to a convenient location.

   `git clone git@github.com:UNSWComputing/PyBullet-Nao-Motion-Editor.git`
2. Create a virtual environment. _Note: Make sure you have `python 3.7.5` or above installed._

   `python -m venv qibullet`
3. Activate the environment.

   `source /qibullet/bin/activate` on Linux
   
   `.\qibullet\Scripts\activate` on Windows
4. Install the required python modules.

   `pip install -r requirements.txt`

## Usage
Before you use the motion editor, run `python test-script.py` to ensure everything works. You may need to agree to Softbank's Licence agreement in the terminal the first time you run it.

To use the motion editor, run `python move-joints.py` and wait for the Nao model and sliders to load in the GUI.

You can now adjust the sliders to move the Nao model's joints.

To capture a keyframe, click the capture keyframe button(scroll down to the bottom of all the sliders). This records the joint values internally.

To generate a `.pos` file, close the pybullet window or hit `ESC`. A file with the name `test-motion-2.pos` should get generated.

## Notes
- This is still a work in progress and hasn't been tested sufficiently. So the current interface/workflow might be a bit janky.
