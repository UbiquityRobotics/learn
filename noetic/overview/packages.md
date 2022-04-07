---
title: "Package Repositories"
permalink: noetic_package_repositories
group: overview
rosver: noetic
nav_order: 3
nav_exclude: false
---

# APT Package Repositories

#### ⮞ [Ubiquity Repo for Ubuntu](https://packages.ubiquityrobotics.com/ubuntu/ubiquity)


#### ⮞ [Ubiquity Testing Repo for Ubuntu](https://packages.ubiquityrobotics.com/ubuntu/ubiquity-testing)

As you can see we only build for Ubuntu, beacuse that is what we use and support, but this might change in the future.

### Adding the Repositories:

    Add the source to your apt lists:
    sudo sh -c 'echo "deb https://packages.ubiquityrobotics.com/ubuntu/ubiquity xenial main" > /etc/apt/sources.list.d/ubiquity-latest.list'
    Add our signing key to your trusted list:
    sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C3032ED8
    sudo apt-get update
    
You added our repository! Now you can install our stuff.

If you wish to use the testing repository, replace the first step with:

    sudo sh -c 'echo "deb https://packages.ubiquityrobotics.com/ubuntu/ubiquity-testing xenial main" > /etc/apt/sources.list.d/ubiquity-latest.list'

### About The Testing Repositories

As packages are built they are added to the build server local repo, which is not publicly accessible. When a build 'session' is over, the packages are uploaded into the ubiquity-testing repository for that distro. When is is determined to be stable enough, it is merged into the main ubiquity repository. Unless you are up for wild adventures in non-functional packages, we suggest not using the testing repo.

### Package/Repository Signatures

All Ubiquity repositories are signed with the Ubiquity Robotics signing key, which has the key id `C3032ED8` and fingerprint `113E 2BBB 335C 2C80 32EA F63D 4BBE 95A1 C303 2ED8`. Currently individual packages are not signed, as it is not necessary for [SecureApt](https://wiki.debian.org/SecureApt). This means that if you are downloading packages directly (and not through your package manager), there is no guarantee of package integrity.