to make the lib with python 3, you need to use virtualenv to work in a python3 environment.

$ source [path-to-python-env]/bin/activate
$ mkdir build & cd build
$ cmake -DPYTHON3=ON ..
$ make

