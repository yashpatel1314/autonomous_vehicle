"""pytest configuration — ensures the av_sim package is importable in tests."""
import sys
import os

# When running under colcon test the workspace install is already on sys.path.
# When running manually from the source tree without sourcing setup.bash, add
# the package source directory so imports resolve without a full build.
_pkg_src = os.path.join(os.path.dirname(__file__), '..', '..')
if _pkg_src not in sys.path:
    sys.path.insert(0, os.path.abspath(_pkg_src))