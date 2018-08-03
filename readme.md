# Arc-tracing Experiment scripts - Version that works with the Omega Robot


## What is this?
These are some nifty Python scripts that are the experimental interface for a motor learning task in which participants need to track an arc or other weird shapes with the Omeag robot.


## Installation
Check out the `omega_cpp_py` source and place it in the `robot` subdirectory.

You need `python`, `numpy`, `scipy` and `pygame`.


## Usage

```
python run_arc.py
```

You need two screens - one for the subject and one for the experimenter.


## Experimental design

* 2 familiarisation trials
* 4x50 trials training



### Procedure

Give a seat to the participant

Turn ON the robot

Hold the robot to the most extreme front position and press ‘Reset’

Turn the experiment screen toward the participant and the experimenter screen toward the experimenter

Execute ‘run arc’ on the terminal in the arctrace file

Give the instructions to the participant by showing them the screen

Shut down the light and close the door

‘Load’ the robot, maybe you have to write your keyword in the terminal, close this warning windows

Press the button on the robot to enable the forces before closing the second warning windows

The x is now fixed, check if the participant is in a comfortable position

Replace ‘schedule.txt’ by ‘scheduledemo.txt’

Clik on ‘run’ : the participant has two trials to understand better

Replace the schedule by ‘schedule.txt’

Ask to the participant if he is comfortable, ready, and press ‘run’



### Instructions

In this experiment, you will see two white dots on the screen with a curved line between them. Your task is to move from the one dot to the other while following as closely as possible the white trajectory between them.

On each trial, the robot first moves you to the start position. You then have to hold the handle there until you see the black cursor appear on the screen. Once the black cursor appears, you can control it by moving the robot handle.
You can only move the robot up or sideways, not towards you or away from you.

To end the trial, you have to reach the target area and hold still there. Then the trajectory will turn green.
The target becomes the start position, the start becomes the target  and you follow the shape on the other way.

It is important to keep the same speed during the entire experiment. Each six trials a red cursor will show you the right speed, it will just follow the shape on the screen. While the cursor shows the movement, please do not move. Please try to respect this velocity.

If your movement takes too long, the trial aborts and the screen turns red. Then the robot moves you to the start position of the next trial.

However, in the beginning of the trial, when the cursor appears, you don’t need to start moving right away, but as soon as you start moving, you need to complete the movement in the desired time.

To summarize, your task is to move from the starting dot to the target dot following the curved white line as closely as possible.

You will be tested in 4 blocks of 50 trials.
Do you have any questions?


→ Another thing: we give them a few “familiarisation” trials, right? Make sure you announce this as well.





## TODO

- [x] When you are outside of start at the beginning of a trial, reset the trial
- [x] Cursor never appears when you start moving too early.
- [x] First data point is wrong (from previous trial?)
- [x] Remove demo cursor?
- [ ] Active-to-null should stay within the X-plane
- [ ] Make instruction video
