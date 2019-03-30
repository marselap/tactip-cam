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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/matija-kk/projekt/src/vision_opencv/cv_bridge"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/matija-kk/projekt/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/matija-kk/projekt/install/lib/python2.7/dist-packages:/home/matija-kk/projekt/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/matija-kk/projekt/build" \
    "/usr/bin/python" \
    "/home/matija-kk/projekt/src/vision_opencv/cv_bridge/setup.py" \
    build --build-base "/home/matija-kk/projekt/build/vision_opencv/cv_bridge" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/matija-kk/projekt/install" --install-scripts="/home/matija-kk/projekt/install/bin"
