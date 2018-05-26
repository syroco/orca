<img src="docs/source/_static/orca-b.png?raw=true" alt="ORCA Logo" width="200">

# Optimisation-based framework for Robotic Control Applications

| Linux/OSX        | Windows  |
| ------------- |:-------------:|
| [![Build Status](https://travis-ci.org/syroco/orca.svg?branch=master)](https://travis-ci.org/syroco/orca)     | [![Build status](https://ci.appveyor.com/api/projects/status/vq4jxmcqmtjgom1x/branch/master?svg=true)](https://ci.appveyor.com/project/ahoarau/orca/branch/master) |

## Dependencies

* A modern **c++11** compiler (gcc > 4.8 or clang > 3.8)
* **cmake** > 3.1
* **iDynTree** (optional, shipped)
* **qpOASES** 3 (optional, shipped)
* **Eigen** 3 (optional, shipped)
* **Gazebo** 8 (optional)

ORCA is self contained ! It means that is ships **iDynTree** and **qpOASES** inside the project, allowing fast installations and easy integration on other platforms.



## Installation

> Always keep in mind that it's better to install the dependencies separately if you plan to use **iDynTree** or **qpOASES** in other projects. For now only **iDynTree** headers appear in public headers, but will be removed eventually to ease the distribution of this library.

### Installation with embedded dependencies

This is the easiest installation if you have a clean system and want to test quickly :


```bash
git clone https://github.com/syroco/orca
cd orca
mkdir build ; cd build
cmake ..
cmake --build . # or just 'make'
```

### Installation without embedded dependencies (recommended)

This installation requires to build the dependencies separately.

#### Eigen

```bash
wget http://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2
tar xjvf 3.3.4.tar.bz2
cd eigen-eigen-dc6cfdf9bcec
mkdir build ; cd build
cmake --build .
sudo cmake --build . --target install
```

#### qpOASES

```bash
wget https://www.coin-or.org/download/source/qpOASES/qpOASES-3.2.1.zip
unzip qpOASES-3.2.1.zip
cd qpOASES-3.2.1
mkdir build ; cd build
cmake .. -DCMAKE_CXX_FLAGS="-fPIC"
cmake --build .
sudo cmake --build . --target install
```

#### iDynTree

```bash
git clone https://github.com/robotology/idyntree
cd idyntree
mkdir build ; cd build
cmake ..
cmake --build .
sudo cmake --build . --target install
```

#### Gazebo

Examples are built with Gazebo 8. They can be adapted of course to be backwards compatible.

```bash
curl -ssL http://get.gazebosim.org | sh
```

> NOTE: You can almost always avoid calling sudo, by calling `cmake .. -DCMAKE_INSTALL_PREFIX=/some/dir` and exporting the `CMAKE_PREFIX_PATH` variable : ` export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/some/dir`
