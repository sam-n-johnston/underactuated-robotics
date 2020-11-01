# Underactuated robotics assignments - MIT 6.824

## Prerequesits

You need to have docker installed. These instructions work for mac.

## Set 1

To start set 1, run:

```
./drake_docker_utility_scripts/docker_run_notebook.sh drake-20190508  ./set_1/
```

## Set 2

To start set 2, run:

```
./drake_docker_utility_scripts/docker_run_notebook.sh drake-20190508  ./set_2/
```

## Set 3

To start set 3, run:

```
./drake_docker_utility_scripts/docker_run_notebook.sh drake-20190508  ./set_3/
```

## Set 4

To start set 4, run:

```
./drake_docker_utility_scripts/docker_run_notebook.sh drake-20190508  ./set_4/
```

## Set 5

To start set 5, run:

```
./drake_docker_utility_scripts/docker_run_notebook.sh drake-20190508  ./set_5/
```

## Project

To start the project, run:

```
docker build ./project --tag drake_and_tools
docker run -it -p 8080:8080 -v "$(pwd)/project"":"/notebooks --name underactuated_robotics_project drake_and_tools
```

To run the tests, go in the docker container and run `python3 ./TestTakeOffPlus.py`. You can also open the jupyter notebook `unit_tests` and run them from there to get some information in plots about the tests.

The objective of the project was to have a more precise robot that could, on the very first jump, get very close to the desired settings (desired height and lateral speed). To do this, I used a sort of simulation of the robot (see `get_touchdown_beta_for_liftoff_beta`). This tried to find the correct touchdown beta and bottom l_rest to reach the desired settings. The problem of using this approach is that, in it's current implementation, it is very sensitive to incorrect parameters (mass, moment of inertia, etc.).

Conclusion:

There were a few problems with the simulation:

- The mass of the robot was incorrect based on the potential energy in the simulation. The mass, which is defined as 1kg in the sdf, is actually ~3.09 in the simulation based on the CalcPotentialEnergy of drake.
- The moment of inertia is incorrect. To find this out, I ploted the accelerations with the current angle of beta to find out that it's about 60% more than what is expected. I didn't figure out how the simulation calculates the inertia of the robot yet.

Because of this, it has been hard to get all tests passing. It also caused the normal simulations problems because the algorithm is sensitive to incorrect parameters, as mentioned.

If I were to try again, I would suggest:

- Try implementing the same algorithm, but adding a step that learn the parameters over time
- Go in the literature to find other solutions.

If I were to design an algorithm for the real world, where knowing the current state of the robot is hard or imprecise, I would:

- Check in the literature
- Think of a new algorithm that doesn't take into account any precise physics, but learns over time to improve itself, like transfer learning, reinforcement learning. The same way that a human can do a backflip, but doesn't need to know his mass, moment of inertia, etc.
