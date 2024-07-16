An application for running QOC on iOS.



## iQOC

Tested with the following combinations:

- XCode 12.0, iOS SDK14.2



## Example Usage

- iQOC is used to replay QOC capture file on iOS.
- To replay the capture file:
  - Download the container of iQOC from *XCode->Window->Devices and Simulators*. Find the installed APPs, and then download the iQOC container.
  - Put the capture file into the *${DOWNLOADED_CONTAINER}/AppData/Documents/*
  - Replace container in *XCode->Window->Devices and Simulators*
  - download the container to see the generated viewer
- Start iQOC, the QOC rasterized depth map will be displayed.


- Code: iQOCWrapper
- Lib: Replace latest libSDOC.quic.a (built from iOS Build the static library) to QOC.quic

## Update iOS Framework

- The QOC dynamic framework can be found at *{PROJECT_DIR}/iQOC/QOC/lib/QOC.quic.framework*, and developer can replace it at any time.









