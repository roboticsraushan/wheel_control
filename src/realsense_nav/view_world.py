"""DEPRECATED: This wrapper has been removed. Use the package module
`realsense_nav.view_world` (installed) or the script at
`src/realsense_nav/realsense_nav/view_world.py`.

This file is kept as a placeholder to avoid accidental imports. Importing
it will raise an informative error.
"""

def main(*args, **kwargs):
    raise ImportError(
        "Deprecated: use 'realsense_nav.view_world' module (package) or run the installed console script 'ros2 run realsense_nav view_world'"
    )


if __name__ == '__main__':
    main()
