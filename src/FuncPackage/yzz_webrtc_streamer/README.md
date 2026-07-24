# yzz_webrtc_streamer

ROS 2 Jazzy migration of the ROS1 WHIP/WebRTC publisher.  It subscribes to a
ROS image topic and remains idle until `/videostream_push_status` publishes
`std_msgs/msg/Int32(data=1)`.  `data=0` stops the stream.

The default source is the PTZ camera topic `/usb_cam/image_raw`.  To reproduce
the original pest-camera source, launch with:

```bash
ros2 launch yzz_webrtc_streamer yzz_webrtc_streamer.launch.py \
  image_topic:=/usb_cam_2/image_raw_2
```

Runtime Python dependencies are `aiortc`, `av`, and `requests`.  They are kept
outside the ROS package so the package can build before those optional runtime
dependencies are installed.
