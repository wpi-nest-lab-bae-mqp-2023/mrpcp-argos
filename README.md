# MRPCP ArGoS

Make sure you have ArGoS, curl and [this json library](https://github.com/nlohmann/json) installed.

This is a simple diffusion example that calls an HTTP endpoint, modifies the response, and prints it in init.

To build from the **root of the directory**:

```
sudo apt update && sudo apt upgrade && sudo apt install curl && sudo apt-get install libcurl4-openssl-dev
pip install cget && cget install nlohmann/json && cget install fmtlib/fmt
# $CGET_PREFIX is the path of cget (probably /home/user/cget)
mkdir build; cd build; cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=$CGET_PREFIX/cget/cget.cmake -DCMAKE_INSTALL_PREFIX=$CGET_PREFIX ..; make; cd ..
```

To run from the **root of the directory**:

```
argos3 -c experiments/diffusion1.argos
```

