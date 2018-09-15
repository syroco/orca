<img src="docs/source/_static/orca-b.png?raw=true" alt="ORCA Logo" width="200">

# Optimization-based framework for Robotic Control Applications

| Linux/OSX        | Windows  |
| ------------- |:-------------:|
| [![Build Status](https://travis-ci.org/syroco/orca.svg?branch=master)](https://travis-ci.org/syroco/orca)     | [![Build status](https://ci.appveyor.com/api/projects/status/vq4jxmcqmtjgom1x/branch/master?svg=true)](https://ci.appveyor.com/project/ahoarau/orca/branch/master) |

## Dependencies

* **c++11** compiler (gcc > 4.8 or clang > 3.8)
* **cmake** > 3.1
* **Eigen** > 3.2 (optional, shipped)
* **Gazebo** (optional)

### Installation using a catkin workspace

#### Gazebo (Optional)

Examples are built with Gazebo 9. They should be backwards compatible.

```bash
curl -ssL http://get.gazebosim.org | sh
```
#### ORCA
```bash
git clone https://github.com/syroco/orca
cd orca
mkdir build ; cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
# cmake --build . --target install
```
