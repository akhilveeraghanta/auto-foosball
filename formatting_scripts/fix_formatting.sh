#!/bin/bash

#
# This script is used for running formatting checks in CI
#

# The version of the clang executable to use
export CLANG_VERSION=7.0

# The directory this script is in
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# The root directory
ROS_ROOT_DIR="$CURR_DIR/../src"

# Extensions to check formatting for clang-format
CLANG_FORMAT_EXTENSIONS=(h cpp c hpp tpp ino)

# Function to run clang format
function run_clang_format () {
    echo "Running clang format over all files..."
    cd $ROS_ROOT_DIR

    # Generate extension string
    # Formatted as -iname *.EXTENSION -o
    EXTENSION_STRING=""
    for value in "${CLANG_FORMAT_EXTENSIONS[@]}"
    do
        EXTENSION_STRING="$EXTENSION_STRING -name *.$value -o"
    done

    # Find all the files that we want to format, and pass them to
    # clang-format as arguments
    # We remove the last -o flag from the extension string
    find $CURR_DIR/../src/ ${EXTENSION_STRING::-2}  \
        # TODO improve this, shouldn't be hardcoded, make similar to CLANG_FORMAT_EXTENSIONS
        -not -path $CURR_DIR/../src/simulator/external/Box2D-cmake \
        | xargs -I{} -n1000 $CURR_DIR/clang-format-$CLANG_VERSION -i -style=file

    if [[ "$?" != 0 ]]; then
        # There was a problem in at least one execution of clang-format
        exit 1
    fi
}

# Run formatting
run_clang_format

exit 0
