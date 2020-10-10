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
