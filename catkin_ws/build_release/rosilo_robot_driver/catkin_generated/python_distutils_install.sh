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

echo_and_run cd "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_robot_driver"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/yuki/git/ctyuen2022/catkin_ws/install_release/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/yuki/git/ctyuen2022/catkin_ws/install_release/lib/python3/dist-packages:/home/yuki/git/ctyuen2022/catkin_ws/build_release/rosilo_robot_driver/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/yuki/git/ctyuen2022/catkin_ws/build_release/rosilo_robot_driver" \
    "/usr/bin/python3" \
    "/home/yuki/git/ctyuen2022/catkin_ws/src/rosilo/rosilo_robot_driver/setup.py" \
     \
    build --build-base "/home/yuki/git/ctyuen2022/catkin_ws/build_release/rosilo_robot_driver" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/yuki/git/ctyuen2022/catkin_ws/install_release" --install-scripts="/home/yuki/git/ctyuen2022/catkin_ws/install_release/bin"