# wasteTracer
GPU accelerated path tracer using Metal

This program lets the user load in an obj file of their choice and this program will render a path traced image of it.
Optionally, this image can be exported back to disk as a png.

Features:
Tracing of the mesh is accelerated using a KD-tree built with SAH heuristic in O(N * log^2 N).
The supported material model is Blinn-Phong.

Todo:
Implement texture support.
Further optimize tree traversal.
Implement normal interpolation.
Support importing an environment map.

