# CONVINCE coverage-plan
This repo contains the source for the coverage planner in CONVINCE WP3.

Author and maintainer: Charlie Street - `cstreet@robots.ox.ac.uk`

## Dependencies

* [Boost](https://linux.how2shout.com/how-to-install-boost-c-on-ubuntu-20-04-or-22-04/)
* [Catch](https://github.com/catchorg/Catch2)
* [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) 

Eigen may not work after installing via apt. You may need to create a symlink in `/usr/include`:

```
cd /usr/include
sudo ln -sf eigen3/Eigen Eigen
sudo ln -sf eigen3/unsupported unsupported
```


## Building & Installation

In repo root dir, run the following:

```
cmake -S . -B build
cmake --build build
```

## Examples

Upon building, example files can be found in the `build/apps` directory.