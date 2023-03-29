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

echo_and_run cd "/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/src/turtlebot3_autorace_2020/turtlebot3_autorace_driving"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/install_isolated/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/install_isolated/lib/python3/dist-packages:/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/build_isolated/turtlebot3_autorace_driving/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/build_isolated/turtlebot3_autorace_driving" \
    "/usr/bin/python3" \
    "/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/src/turtlebot3_autorace_2020/turtlebot3_autorace_driving/setup.py" \
     \
    build --build-base "/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/build_isolated/turtlebot3_autorace_driving" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/install_isolated" --install-scripts="/home/gbbyrd/Desktop/Code/School/Aue823_Spring22_Team1/catkin_ws/install_isolated/bin"
