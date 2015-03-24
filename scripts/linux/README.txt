INSTRUCTIONS FOR SCRIPTS DIRECTORY

  1. INSTALLATION SCRIPTS

    1.A) MADARA for AR.Drone 2.0

      build_madara_drone.sh automates the downloading and building of
      ACE and MADARA for the AR.Drone 2.0 using the toolchain pointed
      to by the $ARM_ROOT variable. Usage of the the script is
      the following:

      ./build_madara_drone.sh <location>

      location is an optional parameter that dictates where the ace
      and madara directories will be created for the repositories. By
      default, location is set to your $HOME directory.

      It is recommended that you run the following script after you
      build, in order to strip the binaries for quicker transfer to
      the drones.

      ./strip_arm_binaries.sh

      To ftp useful files to the drone and set the appropriate permissions
      for using MADARA and ACE, use the following script

      ./ftp_push_binaries.sh <ip>

      ip is the ip of the drone to push to. By default, this ip is set to
      192.168.1.1, which is the default ip of any drone that has just been
      booted up. This ip will change to something else once you have called
      the swarmSetup.sh script on the drone. After swarmSetup.sh call on
      drone10, for instance, you would need to call the following to push
      binaries to the drone on the swarm network SSID.

      ./ftp_push_binaries.sh 192.168.1.10

    1.B) MADARA for host architecture

      build_madara.sh automates the downloading and building of ACE and
      MADARA for the host architecture using g++ and makefiles. Usage of
      the script is the following:

      ./build_madara.sh <location>

      location is an optional parameter that dictates where the ace and
      madara directories will be created for the repositories. By default,
      the location is set to your $HOME directory. 

  2. Setting up a Parrot AR.Drone 2.0

    2.1 Hosts

      We have provided a convenience hosts file that allows for addressing
      other drones as drone{x}, where x is the last byte of the IPV4 address.
      This file is uploaded during usage of the ftp_push_binaries.sh script.

    2.2 swarmSetup.sh

      To operate as a swarm, each drone must reset its wireless adapter to
      Wifi-adhoc mode and join the "swarm" SSID. We provide the swarmSetup.sh
      script to automate that process. This script, as with the hosts file,
      is uploaded during usage of the ftp_push_binaries.sh script. The usage
      of this script is the following (from the data/video directory on the
      drone):

      ./swarmSetup.sh ath0 swarm <ip>

      Where ip is the intended ip address of the drone. Convention dictates
      the usage of 192.168.1.{drone_number}, where drone_number is the number
      on an orange label attached to the drone. This drone_number is also
      included in the default wireless access point name, in case the label
      falls off.

    2.3 Library symbolic links

      For new drones, you will likely have to create symbolic links from the
      /lib directory to the ACE and MADARA libraries that you've uploaded with
      ftp_push_binaries.sh. To do this, type the following at a command line
      from the drone (available via telnet):

      cd /etc
      ln -s /data/video/libACE.so libACE.so
      ln -s /data/video/libMADARA.so libMADARA.so

      This operation should only be necessary once, per drone

    2.4 Setting the hostname

      By default, MADARA assumes that a unique hostname has been set for
      the drone. If we don't set one, we'll have to set the hostname each
      time we invoke the application or risk packets being dropped because
      multiple drones are claiming to be the same drone. To fix this, change
      the hostname in the following ways (from a command line on the drone):

      cd /etc
      echo drone{drone_number} > hostname
      hostname drone{drone_number}

      Where drone_number is the number that identifies the drone, i.e., if
      the intended ip address of the drone is 192.168.1.25, then we should
      do the following:

      cd /etc
      echo drone25 > hostname
      hostname drone25

      This operation only has to be executed once. Afterwards, the change to
      the /etc/hostname will be permanent, unless the drone's harddrive is
      flashed/deleted.


