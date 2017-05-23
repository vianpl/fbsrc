# fbsrc: frame buffer to v4l2 capture device driver

Works great with gstreamer, to test it run:

- On the fb side:
```
sudo gst-launch-1.0 -v videotestsrc ! capsfilter caps=video/x-raw,framerate=60/1 ! fbdevsink device=/dev/fbX
```
- On the v4l2 device side:
```
gst-launch-1.0 v4l2src device=/dev/videoX ! capsfilter caps=video/x-raw,framerate=60/1 ! glimagesink
```
To get the video and fb device numbers check dmesg.
