# Arduino Code

This folder in the repository holds all of the Arduino sketches for this project

## demo2_new.ino

This is the original project file. Our goal with this file was to slowly spin the robot until it was properly aligned with the beacon. The team invested a lot of effort into trying to make this approach work. However, it was determined to be unfeasible for a couple of reasons. The first is the slow refresh rate on the camera. Second, the Pi was not able to process and send the information to the Arduino fast enough. This was a frustrating false start, but it gave us a decent starting place. We decided to create a new file to try a new approach, which is seen below labelled as demo2_new.ino.

## demo2_new.ino

This is the new and revised version of the Arduino code. The team abandoned the slow turning method for the robot. We instead opted to have the robot turn 45 degrees at a time and then pause for long enough to receive good data from the camera. We also decided to structure this code like a finite state machine. This worked really well because it made it simple to communicate the data that was needed from the Pi using the state variable in the Arduino code. This code is used for the second demonstration task, which involves moving to the robot and circling it.

## demo2_noCircle.ino

This is the code for the first task in the demonstration. It is the saw as demo2_new.ino but without the third state (which is used to circle around the beacon). The files were structured this way to ensure no changes for the second demo task would negatively impact the work we already had. This made sure we could isolate each task and ensure it was working properly.
