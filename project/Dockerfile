FROM robotlocomotion/drake:bionic-20201009
WORKDIR /notebooks
RUN apt-get -y update
RUN apt-get -y install python3-pip
RUN pip3 install numpy
CMD ["/bin/bash", "-c", "cd /notebooks && jupyter notebook --ip 0.0.0.0 --port 8080 --allow-root --no-browser"]
