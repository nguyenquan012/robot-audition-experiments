# robot-audition-experiments
Conducting experiments for robot audition to localize sound sources and record data from robots and the sound environment.

See one recorded video of the experiment related to the code from this link https://www.youtube.com/watch?v=DU2-v_EGCNU.

The purpose of these codes is to make iterations of moving the robot from a starting point A to a final point B by following a predefined path, then automatically returning to A. At A, it automatically tunes its position and orientation as the last round, then starts a new loop.
On the way to B, the robot does source localization and records all the data related to audio signal, and robot over time. After reaching the end point B, the system stops source localization and recording data.
These codes make sure that the robot trajectory is repeated in all experiments.

I'm lazy to manually start new experiment each time it is finished, and because there are a lot of experiments to run (running with different speed mode, changing sound environment,..), so I made a shell script file to manage them all automatically.

To read the code. Please start from the shell script file: src\hark_turtlebot_fdnetworks\experiment\experiment.sh.

In the video, it shows one iteration of experiment when there is a moving source (which I put a loudspeaker on another turtlebot), music, and noises.
