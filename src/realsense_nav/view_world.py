"""
Light wrapper module so the console_script entry point
`realsense_nav.view_world:main` works while the implementation
lives under `realsense_nav.scripts.view_world`.
"""
def main():
    # Import locally to avoid import-time side-effects
    from realsense_nav.scripts.view_world import main as _main
    return _main()


if __name__ == '__main__':
    main()
