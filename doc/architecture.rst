Architecture and Process Flow
=============================

The topmost object in LSD-SLAM is :class:`lsd_slam::SlamSystem`.  It is the
primary container of both system state and the sub-threads which do the work.
Once created, the main access point is :func:`lsd_slam::SlamSystem::nextImage`
(for mono) and :func:`lsd_slam::SlamSystem::nextImageSet` (for stereo)
which are used to provide the next image to the algorithm.

The main data structures are:

:class:`lsd_slam::Frame`
  is a single image from the incoming data stream.  It contains pre-computed
  values (for example, an image pyramid, gradient image, etc.) used when
  processing that image.   Frames are generally not retained, unless an
  image is converted to a KeyFrame

:class:`lsd_slam::KeyFrame`
  represents a single key frame.  It contains the original Frame, along with
  supplemental data structures used to perform KeyFrame-specific functions:
  a DepthMap for the associated point cloud, and a TrackingReference for
  accelerating subsequent tracking operations against the Key Frame.

:class:`lsd_slam::DepthMap`
  contains the data and code for performing KeyFrame point cloud updates
  (virtual baseline stereo).

:class:`lsd_slam::KeyFrameGraph`
  stores all KeyFrames both as a simple list and as an undirected pose graph.

LSD-SLAM has four top-level threads.  The thread objects are designed as
execution contexts first, although they also store the subset of the
"global" state which is most relevant to their mission.

:class:`lsd_slam::TrackingThread`
  is responsible for calculating the pose of the current image relative to the
  current Keyframe.

:class:`lsd_slam::MappingThread`
  takes tracked frames and uses them to update the depth map estimate for the
  current KeyFrame.

:class:`lsd_slam::ConstraintSearchThread`
  operates in spare cycles to search for additional cross-links between
  KeyFrames based on appearance.

:class:`lsd_slam::OptimizationThread`
  operates in spare cycles to globally optimize the pose graph.


Each thread object is implemented as an
(ActiveObject)[https://github.com/apl-ocean-engineering/libactiveobject],
a work queue model where method calls to the object are executed
asynchronously in a separate thread of execution.




.. image:: _static/uml/test.svg
