# Changelog
## cam2image_vm2ros

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/)

## [2024.3.1] - 2025-04-30

### Added
- Changelog for host2vm
- Added `throttle_rate` as param. Messages using this will only publish once every `n` ms, or longer. Depends on when message comes in.

### Changed
- Readme for host2vm

##  [2024.3] - 2025-01-31

### Added
- Added Rotation for when on RELbot
- Added frame-check for when on RELbot
- Added parameters to config.


### Changed 
- Updated the `-h` command output


##  [2024.0] - 2025-01-31

Updated version for (A)SDfR course in academic year '24/'25

### Added
 - Added CI runner for auto-packaging on Git
 - Added timeout to config file.
 - Added readme

### Changed
 - Renamed multipass_environment to vm2ros for more descriptive package names
 - Update authors in package.xml
 - Reduce logging output

### Removed
- Removed unused files leftover from image_tools. 
 


### All information below is from GIT history. Pieces of information might be missing
## Image tools SDfR multipass_environment
##  [v1.0] - 2024-04-02

 - Stable working version. Most of this package is derived from the ROS package `image_tools`

### Added
 - Can receive images from 'host' environment.
