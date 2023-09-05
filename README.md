# PyBullet Nao Playground + Motion Editor
A simulation environment for testing Nao motions/behaviors/vision(possibly?).

## Setup
1. Clone the repository to a convenient location.

   `git clone git@github.com:UNSWComputing/PyBullet-Nao-Motion-Editor.git`
2. Create a [virtual environment][python3-venv]. _Note: Make sure you have `python 3.7.5` or above installed._

   `python -m venv qibullet`
3. Activate the environment.

   `source ./qibullet/bin/activate` on Linux
   
   `.\qibullet\Scripts\activate` on Windows
4. Install the required python modules.

   `pip install -r requirements.txt`

## Running the POS file generator
Before you use the motion editor, run `python test-script.py` to ensure everything works. You may need to agree to Softbank's Licence agreement in the terminal the first time you run it.

To use the motion editor, run `python move-joints.py` and wait for the Nao model and sliders to load in the GUI.

You can now adjust the sliders to move the Nao model's joints.

To capture a keyframe, click the capture keyframe button(scroll down to the bottom of all the sliders). This records the joint values internally. _Note: There is not yet a way to specify the duration or the stiffness of each joint._

To generate a `.pos` file, close the pybullet window or hit `ESC`. A file with the name `motion_<date+time>.pos` should get generated.

## Notes
- This is still a work in progress and hasn't been tested sufficiently. So the current interface/workflow might be a bit janky.

[python3-venv]: https://docs.python.org/3/library/venv.html
