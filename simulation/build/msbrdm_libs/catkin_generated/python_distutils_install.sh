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

echo_and_run cd "/home/pablo/Desktop/welding_robot/simulation/src/msbrdm_libs"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/pablo/Desktop/welding_robot/simulation/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/pablo/Desktop/welding_robot/simulation/install/lib/python3/dist-packages:/home/pablo/Desktop/welding_robot/simulation/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/pablo/Desktop/welding_robot/simulation/build" \
    "/usr/bin/python3" \
    "/home/pablo/Desktop/welding_robot/simulation/src/msbrdm_libs/setup.py" \
     \
    build --build-base "/home/pablo/Desktop/welding_robot/simulation/build/msbrdm_libs" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/pablo/Desktop/welding_robot/simulation/install" --install-scripts="/home/pablo/Desktop/welding_robot/simulation/install/bin"
