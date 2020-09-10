# Project

## Project proposal

Create a 3d hopper similar to this [one](http://www.ai.mit.edu/projects/leglab/robots/3D_hopper/3D_hopper.html) and be able to control it's (x,y) position & speed while maintaining jumping height. It should also be able to control it's hopping height. 

How can I increase the complexity of this project?
- 2D, but find multiple different algos to make it hop, measuring the performance of each. I'll start with this one and then review later
  - Options:
    - Raibert option
    - DNN (model free)
    - DNN (with model) => is this possible?
    - Find other options online
    - Create a new controller?
    - LQR (Is it even applicable here?)
  - Follow performance metrics for each of the options:
    - Time to process 
    - Time to train (if applicable)
    - Success metric:
      - Euclidean error distance
      - Energy use
      - Speed
      - Accuracy
- 2D, but with noise
- 3D
- Acrobatics (flips, etc.)

Other phases of things I could do later:
- Create the robot in the real world
- Can we create a DNN to improve simulation speed? Specifically with contact?
