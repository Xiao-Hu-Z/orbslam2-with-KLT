# AGENTS.md

## Cursor Cloud specific instructions

This is a C++ ORB-SLAM2 project enhanced with KLT optical flow tracking. It builds with CMake and has no package manager lockfiles.

### Build toolchain

- **Compiler**: Must use `g++-11` (GCC 11). The system default compiler (clang 18) lacks the C++ standard library headers needed. GCC 12+ triggers a `static_assert` in `std::map` due to an Eigen `aligned_allocator` type mismatch in `include/LoopClosing.h`.
- **Eigen**: The project uses Eigen 3.3.7 installed at `/usr/local/include/eigen3`. The system Eigen 3.4.0 at `/usr/include/eigen3` is incompatible.
- **Pangolin**: Built from source (v0.6) and installed at `/usr/local/lib/libpangolin.so`. Requires `CMAKE_CXX_FLAGS="-include cstdint"` when building with GCC 11+.
- **OpenCV**: System OpenCV 4.6.0 is used. A compatibility header at `/usr/local/include/opencv/cv.h` bridges the deprecated `opencv/cv.h` include used by `include/ORBextractor.h`.

### Build commands

Full clean build from workspace root:

```bash
export CC=gcc-11 CXX=g++-11

# 1. Extract vocabulary
cd Vocabulary && tar zxvf ORBvoc.tar.gz && cd ..

# 2. Build DBoW2
cd Thirdparty/DBoW2 && mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++-11 -DCMAKE_C_COMPILER=gcc-11 && make -j$(nproc) && cd ../../..

# 3. Build g2o
cd Thirdparty/g2o && mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++-11 -DCMAKE_C_COMPILER=gcc-11 -DEIGEN3_INCLUDE_DIR=/usr/local/include/eigen3 && make -j$(nproc) && cd ../../..

# 4. Build ORB_SLAM2
mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++-11 -DCMAKE_C_COMPILER=gcc-11 -DEIGEN3_INCLUDE_DIR=/usr/local/include/eigen3 && make -j$(nproc) && cd ..
```

### Running

Executables are output to `build/`. They require `LD_LIBRARY_PATH` to include the library paths:

```bash
export LD_LIBRARY_PATH=/workspace/lib:/workspace/Thirdparty/DBoW2/lib:/workspace/Thirdparty/g2o/lib:/usr/local/lib:$LD_LIBRARY_PATH
```

Example (RGB-D with TUM dataset):
```bash
./build/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml <path_to_dataset> <path_to_association_file>
```

### Key gotchas

- The Pangolin viewer requires an X11 display (`DISPLAY=:1` is available in Cloud Agent VMs).
- The `opencv/cv.h` compatibility shim at `/usr/local/include/opencv/` is essential; without it, `include/ORBextractor.h` won't compile against OpenCV 4.
- `include/LoopClosing.h` has a fix: `const KeyFrame*` changed to `KeyFrame *const` in the `Eigen::aligned_allocator` template to match `std::map::value_type`.
- `CMakeLists.txt` has an added line to force-include `opencv2/imgcodecs/legacy/constants_c.h` for `CV_LOAD_IMAGE_UNCHANGED`.
- No automated test suite exists; validation is done by running example executables against camera datasets (TUM, KITTI, EuRoC).
- No linter is configured for this C++ project.
