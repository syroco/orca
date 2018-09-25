<img src="docs/source/_static/orca-b.png?raw=true" alt="ORCA Logo" width="200">

## Optimization-based framework for Robotic Control Applications

| Platform | Build status |
| --- | --- |
| Linux / macOS | [![Build Status](https://travis-ci.org/syroco/orca.svg?branch=master)](https://travis-ci.org/syroco/orca) |
| Windows | [![Build status](https://ci.appveyor.com/api/projects/status/vq4jxmcqmtjgom1x/branch/master?svg=true)](https://ci.appveyor.com/project/ahoarau/orca/branch/master)
 
## Dependencies

* **c++14** compiler (gcc > 5 or clang > 4)
* **cmake** > 3.5
* **Eigen** > 3.2 (optional, shipped)
* **Gazebo** (optional)

### Installation using plain cmake

#### Gazebo (optional)

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
```
