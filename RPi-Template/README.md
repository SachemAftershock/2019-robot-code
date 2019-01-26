# Sachem Aftershock Pi Image

Contains the code to build and modify the Raspberry Pi image used for the 2019 competition season. The home repository for this image can be found [here](https://github.com/DanWaxman/pi-gen), and the repository for the original Raspberry Pi image tool can be found [here](https://github.com/RPi-Distro/pi-gen).

## Dependencies and Usage
For an up-to-date and complete set of instructions on how to use this tool, please refer to the [official pi-gen repo](https://github.com/RPi-Distro/pi-gen). 

In short, the script is run on Debian based operating systems, and is executed by running `sudo ./build.sh` -- all of its dependencies are listed in the `depends` file. For the script to run properly, you will require, a `config` file containing the name of your desired image. 

## Modifications
Right now this image is only slightly modified from the official Raspbian image, with all modifications occuring in `stage2`. These changes include:

- Adding some extra tools and dependencies for OpenCV.

- Blacklisted a number of modules pertaining to WiFi and Bluetooth that the default kernel loads.

- Enabled SSH services by default.

- Installed PyNetworkTables and a headless installation of OpenCV.

- Modified the DHCP settings to get a static IP of `10.2.63.25`.

Stages 3+ were deleted as this is strictly a headless image.

## Possible Improvements

Right now, OpenCV is installed using the wheels provided in pip. While this works, it necessitates the installation of a few libraries that we aren't likely to use, adding to the size of the image. Building OpenCV manually would help with this.

Right now, for the sake of development the filesystem is read-write. However, we're likely to sporadically cut power to the Raspberry Pi in build season and competition -- making the filesystem read-only would help mitigate possible corruption of the SD card (though not eliminate it). 
