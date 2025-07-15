#!/usr/bin/env python3
import os
import shutil
import subprocess
import tarfile

def main():
    # Set the working directory to the root of the repository
    os.chdir(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

    # Remove the dist directory if it exists
    if os.path.exists("dist"):
        shutil.rmtree("dist")

    # Build the project
    subprocess.run(["colcon", "build"], check=True)

    # Create a tarball of the install directory
    with tarfile.open("dist/release.tar.gz", "w:gz") as tar:
        tar.add("install", arcname=os.path.basename("install"))

if __name__ == "__main__":
    main()
