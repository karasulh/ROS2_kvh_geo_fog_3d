#!/bin/bash

echo `pwd`

# export ROSDISTRO_INDEX_URL=https://gitlab.mitre.org/dart-release/rosdistro/raw/dart/index-v4.yaml
# export PYTHONHTTPSVERIFY=0

. clean_deb.sh

echo "Building Debian..."
bloom-generate rosdebian --os-name ubuntu --os-version xenial --ros-distro kinetic
bash packaging/fix_rules.sh
fakeroot debian/rules binary

unset ROSDISTRO_INDEX_URL
unset PYTHONHTTPSVERIFY
