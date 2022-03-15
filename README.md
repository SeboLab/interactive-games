# Benefits of an Interactive Robot Character in Immersive Puzzle Games

Robot control code for the Interactive Robot Character in Immersive Puzzle Games study

Created by [Ting-Han Lin](mailto:tinghan@uchicago.edu) and [Spencer Ng](mailto:spencerng@uchicago.edu) at the [Human-Robot Interaction Lab](https://hri.cs.uchicago.edu) at the University of Chicago.

## Setup

1. Install PyQt5
2. Generate the PyQt GUI: `pyuic5 wizard.ui -o wizardui.py`
3. Install the Anki Vector SDK and configure the Anki Vector robot with your computer: `python3 -m anki_vector.configure`

## Wizard Usage

1. Launch ROS with `roscore`
2. Run the wizard for either the robot (default) condition or the human condition: `python3 main.py [--human]`
3. The wizard will launch. Upon completing a study, click "export data" to generate a participant JSON file in `data/`
