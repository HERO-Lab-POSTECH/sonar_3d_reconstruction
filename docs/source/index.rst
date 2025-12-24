Sonar 3D Reconstruction
=======================

Real-time probabilistic 3D underwater terrain mapping system using Oculus multibeam sonar and Fast-LIO odometry.

Features
--------

- **IWLO Probability Update**: Intensity-Weighted Log-Odds Bayesian mapping
- **C++ Backend**: OpenMP parallel processing for 10x speed improvement
- **Out-of-Core Storage**: Disk-based tile storage for large-scale mapping
- **Cross-talk Filter**: Multibeam sonar noise reduction

.. toctree::
   :maxdepth: 2
   :caption: Getting Started

   readme

.. toctree::
   :maxdepth: 2
   :caption: Design Documents

   design/iwlo_design
   design/octree_mapping
   design/outofcore_design
   design/parameter_centralization
   troubleshooting

.. toctree::
   :maxdepth: 2
   :caption: API Reference

   api/config
   api/3d_mapper
   api/3d_mapper_node
   api/mapper_backend
   api/probability_updater
   api/outofcore_tile_mapper
   api/iwlo_updater
   api/voxel_storage
   api/octree_storage
   api/octree_mapper
   api/tile
   api/tile_manager
   api/utilities


Indices and tables
==================

* :ref:`genindex`
* :ref:`search`
