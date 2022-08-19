# Drone simulator

## Docs
- [Docs](docs)
- http://www.mymathlib.com/diffeq/second_order/

## Build
### Install the Chrono library
```
git clone https://github.com/projectchrono/chrono --recurse-submodules
sudo apt-get update
sudo apt-get install -y libeigen3-dev libirrlicht-dev libopenmpi-dev libthrust-dev libxxf86vm-dev libgl1-mesa-dev freeglut3 freeglut3-dev ninja-build
cd chrono/
mkdir build && cd build
cmake -GNinja -DENABLE_MODULE_IRRLICHT=ON -DENABLE_MODULE_VEHICLE=ON -DENABLE_MODULE_POSTPROCESS=ON ..
cmake --build .
sudo cmake --install .
sudo ldconfig
```

### Build simulator
```
mkdir build && cd build
cmake -GNinja -DCMAKE_BUILD_TYPE=Debug -DChrono_DIR=/usr/local/lib/cmake ..
cmake --build .
```
