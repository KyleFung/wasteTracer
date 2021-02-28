# wasteTracer
GPU accelerated path tracer using Metal

This program lets the user load in an obj file of their choice and this program will render a path traced image of it.
Optionally, this image can be exported back to disk as a png.

Features:
* Loads obj files with only triangular faces
* KD-tree construction using SAH heuristic in O(N * log^2 N).
* Ray tracing executed entirely in GPU.
* Blinn-Phong material model.

Example with a 500k triangle mesh:

64 samples per pixel (526.89s)
![engel64](https://user-images.githubusercontent.com/5490754/109407840-05a92500-7939-11eb-8e6f-6af184308c76.png)

128 samples per pixel (1051.98s)
![engel128](https://user-images.githubusercontent.com/5490754/109407850-135eaa80-7939-11eb-9eaa-84153f6460c7.png)

256 samples per pixel (2093.31s)
![engel256](https://user-images.githubusercontent.com/5490754/109407768-8ae00a00-7938-11eb-9f7b-3b8e34bfa218.png)

Todo:
* Implement texture support.
* Further optimize tree traversal.
* Implement normal interpolation.
* Support importing an environment map.
