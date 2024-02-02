# MRPCP ArGoS

Make sure you have ArGoS and [this json library](https://github.com/nlohmann/json) installed.

This is a simple diffusion example that calls an HTTP endpoint, modifies the response, and prints it in init.

To build from the **root of the directory**:

```
mkdir build; cd build; cmake -DCMAKE_BUILD_TYPE=Release ..; make; cd ..
```

To run from the **root of the directory**:

```
argos3 -c experiments/diffusion1.argos
```

