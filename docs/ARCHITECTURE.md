# Architecture

The pipeline is intentionally split into a deterministic C++/PCL preprocessing stage and a Python/Open3D decision stage. ROS 2 messages isolate the detector from downstream autonomy components. Temporal confirmation reduces isolated false positives, while gabarit intersection limits decisions to the train corridor.
