from multiprocessing import freeze_support

# PyInstaller's multiprocessing override must run before importing the application.
# If a bundled native/third-party geometry path creates worker processes on Windows,
# this diverts those worker invocations instead of recursively starting IFCPathApp.
freeze_support()

from ifcpath.app_runner import main


if __name__ == "__main__":
    main()
