#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/sbpena01/GitHub/RBE521-Quadruped-Robot/src/gazebo_ros_pkgs/gazebo_plugins"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/sbpena01/GitHub/RBE521-Quadruped-Robot/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/sbpena01/GitHub/RBE521-Quadruped-Robot/install/lib/python3/dist-packages:/home/sbpena01/GitHub/RBE521-Quadruped-Robot/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/sbpena01/GitHub/RBE521-Quadruped-Robot/build" \
    "/usr/bin/python3" \
    "/home/sbpena01/GitHub/RBE521-Quadruped-Robot/src/gazebo_ros_pkgs/gazebo_plugins/setup.py" \
     \
    build --build-base "/home/sbpena01/GitHub/RBE521-Quadruped-Robot/build/gazebo_ros_pkgs/gazebo_plugins" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/sbpena01/GitHub/RBE521-Quadruped-Robot/install" --install-scripts="/home/sbpena01/GitHub/RBE521-Quadruped-Robot/install/bin"