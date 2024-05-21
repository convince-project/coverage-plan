# CONVINCE COVERAGE-PLAN
This repo contains the source for the coverage planner in CONVINCE WP3.

## Installation

COVERAGE-PLAN has been tested on Ubuntu 22.04 with CMake 3.22.

### Dependencies

COVERAGE-PLAN requires the following dependencies:

* [Boost](https://linux.how2shout.com/how-to-install-boost-c-on-ubuntu-20-04-or-22-04/) (Tested with 1.82)
* [Catch](https://github.com/catchorg/Catch2) (Tested with 2.13.8-1, installed via `apt install` on Ubuntu 22.04)
* [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)  (Tested with 3.3)
* [Despot](https://github.com/AdaCompNUS/despot) (Tested with API_redesign branch)

Eigen may not work after installing via apt. You may need to create a symlink in `/usr/include`:

```
cd /usr/include
sudo ln -sf eigen3/Eigen Eigen
sudo ln -sf eigen3/unsupported unsupported
```

### Building COVERAGE-PLAN

After installing the dependencies, run the following in the root directory:

```
cmake -S . -B build
cmake --build build
```

## Running Unit Tests
To run the unit tests, run the following in the root directory:
```
cd build/tests
./unitTests
```

## Examples

Upon building, example files can be found in the `build/apps` directory.


## Build the documentation

The COVERAGE-PLAN documentation can be found [here](https://convince-project.github.io/coverage-plan). 
If you want to build it locally, do the following:

1. Install the required packages:

    ```bash
    sudo apt install doxygen
    pip install -r docs/requirements.txt
    ```

3. Build the documentation:

    ```bash
    cd docs
    make html
    ```

4. Look at the documentation:

    ```bash
    cd docs
    firefox build/html/index.html
    ```

### Clean documentation build artifacts

If you want to clean the documentation, you can run:

```bash
cd docs
make clean
```

## Maintainer

This repository is maintained by:

| | | |
|:---:|:---:|:---:|
| Charlie Street | [@charlie1329](https://github.com/charlie1329) |[c.l.street@bham.ac.uk](mailto:c.l.street@bham.ac.uk?subject=[GitHub]%20Coverage%20Plan)|
