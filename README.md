# LSD-SLAM: Large-Scale Direct Monocular SLAM

See my [Development Blog](http://staff.washington.edu/amarburg/site/) for current status.

> __December 2017__   Not as much time as I would like to work on this
over the last year (clearly).   One thing I've discovered is I'm not a huge
fan of Conan.   I ended up making a lot of infrastructure to get what I
wanted out of it --- which was the ability to define dependencies and
have them all built locally.
=======
See my [Development Blog](http://staff.washington.edu/amarburg/site/) for current status.

> So if you've got here, I've thrown out Conan and moved to
[fips](http://floooh.github.io/fips/index.html).   fips ain't perfect, but it
does the job I need it to do.

> __It's good enough I'm deprecating the Conan build.__
If you don't want to use fips, see the `cmake` branch.

> If you really loved the Conan build, the last commit with Conan is [7b8b76f](https://github.com/amarburg/lsd_slam/commit/7b8b76ff6be7e6f4c4eb1576a7f741146eb1bdf4).

> This also means `master` no longer builds Thomas' Pangolin-based GUI.   That's now
in its [own repo](https://github.com/amarburg/lsd-slam-pangolin-gui) which includes this repo.   Think of this repo as the "LSD SLAM Library (w/o a GUI)",
with frontends elsewhere...

This fork started from [Thomas Whelan's fork](https://github.com/mp3guy/lsd_slam) which "relieves the user of the horrors of a ROS dependency and uses the much nicer lightweight [Pangolin](https://github.com/stevenlovegrove/Pangolin) framework instead."

Here is Jakob's original description:

> LSD-SLAM is a novel approach to real-time monocular SLAM. It is fully direct
> (i.e. does not use keypoints / features) and creates large-scale,
> semi-dense maps in real-time on a laptop. For more information see
> [http://vision.in.tum.de/lsdslam](http://vision.in.tum.de/lsdslam)
> where you can also find the corresponding publications and Youtube videos, as well as some
> example-input datasets, and the generated output as rosbag or .ply point cloud.

This repo contains my experiments with LSD-SLAM, for performance, functionality
and structure.   As of November 2016, it diverges significantly from either Jakob
or Thomas's branches in structure (I refactored as a way of learning the code),
but not significantly in terms of functionality (except for all the ways in which
I've broken it in the refactoring).

**master**  is my working / stable-ish branch.   **unstable** is my **really unstable** branch.   **Please note: BOTH BRANCHES ARE MOVING TARGETS.**  it's just that **unstable** is, uh, moving faster.

# Quickstart

My targeted environments are Ubuntu 18.04/16.04,
the [Jetson TX1](http://www.nvidia.com/object/jetson-tx1-module.html) using [NVidia Jetpack 2.3](https://developer.nvidia.com/embedded/jetpack) , and OS X 10.12 with [Homebrew](http://brew.sh/).

__If you want a GUI, start with to [lsd-slam-pangolin-gui](https://github.com/amarburg/lsd-slam-pangolin-gui)__

Common jobs are encoded in `Makefile`.This includes tasks for installing dependencies (in Travis and Docker images for example), and for automating building and testing.

Assuming all of the "standard" (apt-gettable/Brew-able) deps have been installed,

    ./fips gen
    ./fips build

Both configuration (Release vs. Debug) and platform are controlled through
fips settings, so on a linux machine, either

    ./fips gen linux-make-release
    ./fips build linux-make-release

or

    ./fips set config linux-make-release
    ./fips gen
    ./fips build

will build release binaries.  `linux-make-debug` will build debug binaries and the unit tests.

======
# Running

Supports directories or sets of raw images. For example, you can download
any dataset from [here](http://vision.in.tum.de/lsdslam), and run:

    ./fips run LSD -- -c datasets/LSD_machine/cameraCalibration.cfg -f datasets/LSD_machine/images/

I've started to document my performance testing in [doc/Performance.md](doc/Performance.md)

=======
# Docker

For repeatability, builds can occur inside a Docker container.   To do this,
first run `rake docker:image` which is create a local copy of the development Docker image called
`lsdslam-build:local`.   This is a minor iteration on the published `amarburg/lsdslam-dev-host`
image.

Then `rake docker:debug:build` or `rake docker:release:build` which will build the
release in a Docker container up through testing.

This build process will mount and build the current source tree in its own `build_docker-*` tree,
which is not ephemeral.

For now the Docker process is focused on building and testing, not actually running in the Docker image.  Soon enough...

# 5. Related Papers
* **LSD-SLAM: Large-Scale Direct Monocular SLAM**, *J. Engel, T. Schöps, D. Cremers*, ECCV '14

* **Semi-Dense Visual Odometry for a Monocular Camera**, *J. Engel, J. Sturm, D. Cremers*, ICCV '13

# License

LSD-SLAM is licensed under the GNU General Public License Version 3 (GPLv3), see http://www.gnu.org/licenses/gpl.html.

For commercial purposes, we also offer a professional version under different licencing terms.



# 6 Troubleshoot / FAQ

**How can I get the live-pointcloud in ROS to use with RVIZ?**

You cannot, at least not on-line and in real-time. The reason is the following:

In the background, LSD-SLAM continuously optimizes the pose-graph, i.e., the poses of all keyframes. Each time a keyframe's pose changes (which happens all the time, if only by a little bit), all points from this keyframe change their 3D position with it. Hence, you would have to continuously re-publish and re-compute the whole pointcloud (at 100k points per keyframe and up to 1000 keyframes for the longer sequences, that's 100 million points, i.e., ~1.6GB), which would crush real-time performance.

Instead, this is solved in LSD-SLAM by publishing keyframes and their poses separately:
- keyframeGraphMsg contains the updated pose of each keyframe, nothing else.
- keyframeMsg contains one frame with it's pose, and - if it is a keyframe - it's points in the form of a depth map.

Points are then always kept in their keyframe's coodinate system: That way, a keyframe's pose can be changed without even touching the points. In fact, in the viewer, the points in the keyframe's coodinate frame are moved to a GLBuffer immediately and never touched again - the only thing that changes is the pushed modelViewMatrix before rendering. 

Note that "pose" always refers to a Sim3 pose (7DoF, including scale) - which ROS doesn't even have a message type for.

If you need some other way in which the map is published (e.g. publish the whole pointcloud as ROS standard message as a service), the easiest is to implement your own Output3DWrapper.


**Tracking immediately diverges / I keep getting "TRACKING LOST for frame 34 (0.00% good Points, which is -nan% of available points, DIVERGED)!"**
- double-check your camera calibration.
- try more translational movement and less roational movement


