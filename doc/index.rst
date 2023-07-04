.. LSD-SLAM documentation master file, created by
   sphinx-quickstart on Mon Mar 11 09:52:03 2019.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

LSD-SLAM: Large-Scale Direct Monocular SLAM
===========================================

Large-Scale Direct SLAM is a SLAM implementation based on
`Thomas Whelan's fork <https://github.com/mp3guy/lsd_slam>`_ of
Jakob Engel's `repo <https://github.com/tum-vision/lsd_slam>`_ based on
his `PhD research <https://vision.in.tum.de/research/vslam/lsdslam>`_.
As such, this codebase still contains a large portion of Jakob's original
code, particularly for the core depth mapping and tracking, but has been
re-architected significantly for readability and performance.




.. toctree::
   :maxdepth: 2
   :caption: Contents:

   architecture
   api/lsdlsam_root


License
-------

A per Jakob's original code, LSD-SLAM is released under the GPLv3 license.
TU-Munich has offered a licensable version in the past although I am not
aware of the current state of this arrangement.



Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
