## How to install CrowdBag

If you need to use python 3, the source code also sould be installed with python 3. In order to do that, follow theses instructions to activate a virtualenv where python3 is installed:

```
$ source [path-to-python-env]/bin/activate
$ mkdir build & cd build
$ cmake -DPYTHON3=ON ..
$ make
```
