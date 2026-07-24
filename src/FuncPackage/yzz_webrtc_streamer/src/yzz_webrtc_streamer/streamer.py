#!/usr/bin/env python3
"""ROS 2 image to WHIP/WebRTC publisher, migrated from yzz_webrtc_streamer."""

import asyncio
import queue
import random
import re
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


def image_to_bgr(message: Image) -> np.ndarray:
    """Convert the common ROS image encodings without requiring cv_bridge."""
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if message.encoding == 'bgr8':
        return raw.reshape(message.height, message.width, 3)
    if message.encoding == 'rgb8':
        return cv2.cvtColor(raw.reshape(message.height, message.width, 3), cv2.COLOR_RGB2BGR)
    if message.encoding == 'bgra8':
        return cv2.cvtColor(raw.reshape(message.height, message.width, 4), cv2.COLOR_BGRA2BGR)
    if message.encoding == 'rgba8':
        return cv2.cvtColor(raw.reshape(message.height, message.width, 4), cv2.COLOR_RGBA2BGR)
    if message.encoding in ('mono8', '8UC1'):
        return cv2.cvtColor(raw.reshape(message.height, message.width), cv2.COLOR_GRAY2BGR)
    if message.encoding.lower() in ('yuv422', 'yuyv'):
        return cv2.cvtColor(raw.reshape(message.height, message.width, 2), cv2.COLOR_YUV2BGR_YUY2)
    raise ValueError(f'unsupported image encoding: {message.encoding}')


class RosImageTrack:
    """Own the newest ROS frames and expose them to an aiortc video track."""

    def __init__(self, node: Node, width: int, height: int):
        self.node = node
        self.width = width
        self.height = height
        self.frames = queue.Queue(maxsize=3)
        self.last_frame = None

    def image_callback(self, message: Image):
        try:
            frame = image_to_bgr(message)
            if self.width > 0 and self.height > 0:
                frame = cv2.resize(frame, (self.width, self.height))
            stamp = time.strftime('%Y-%m-%d %H:%M:%S')
            cv2.putText(frame, stamp, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3)
            cv2.putText(frame, stamp, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            try:
                self.frames.put_nowait(rgb)
            except queue.Full:
                self.frames.get_nowait()
                self.frames.put_nowait(rgb)
        except Exception as error:
            self.node.get_logger().warning(f'image conversion failed: {error}', throttle_duration_sec=5.0)

    def next_frame(self):
        try:
            self.last_frame = self.frames.get_nowait()
        except queue.Empty:
            pass
        if self.last_frame is None:
            return np.zeros((self.height, self.width, 3), dtype=np.uint8)
        return self.last_frame


class WebRTCStreamer(Node):
    """Wait for platform video commands and publish the selected ROS image stream."""

    def __init__(self):
        super().__init__('webrtc_streamer')
        self._declare_parameters()
        self._load_runtime_dependencies()
        self.streaming = threading.Event()
        self.stop_requested = threading.Event()
        self.loop_thread = None
        self.event_loop = None
        self.peer_connection = None
        self.image_subscription = None
        self.track_source = None
        self.create_subscription(Int32, self.get_parameter('control_topic').value,
                                 self.control_callback, 10)
        self.get_logger().info(
            f'WebRTC streamer ready; waiting for {self.get_parameter("control_topic").value}=1')

    def _declare_parameters(self):
        for name, value in (
            ('image_topic', '/usb_cam/image_raw'),
            ('image_width', 640),
            ('image_height', 480),
            ('fps_max', 30.0),
            ('control_topic', '/videostream_push_status'),
            ('whip_url_base', 'http://43.138.191.89:7208/rtc/v1/whip/?app=live&stream='),
            ('register_number', ''),
            ('request_timeout_sec', 30.0),
        ):
            self.declare_parameter(name, value)

    def _load_runtime_dependencies(self):
        self.runtime_ready = False
        try:
            import requests
            from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
            from av import VideoFrame
            self.requests = requests
            self.RTCPeerConnection = RTCPeerConnection
            self.RTCSessionDescription = RTCSessionDescription
            self.VideoStreamTrack = VideoStreamTrack
            self.VideoFrame = VideoFrame
            self.runtime_ready = True
        except ImportError as error:
            self.get_logger().error(
                f'WebRTC runtime dependency is missing: {error}. Install aiortc, av and requests before streaming.')

    def control_callback(self, message: Int32):
        if message.data == 1:
            self.start_streaming()
        elif message.data == 0:
            self.stop_streaming()

    def whip_url(self):
        base = self.get_parameter('whip_url_base').value
        token = self.get_parameter('register_number').value
        return f'{base}{token}' if base and token else ''

    def start_streaming(self):
        if self.streaming.is_set():
            return
        if not self.runtime_ready:
            self.get_logger().error('Cannot start stream: aiortc/av runtime dependency is unavailable')
            return
        if not self.whip_url():
            self.get_logger().error('Cannot start stream: WHIP URL or device register number is empty')
            return
        self.stop_requested.clear()
        self.streaming.set()
        self.loop_thread = threading.Thread(target=self._run_loop, daemon=True)
        self.loop_thread.start()
        self.get_logger().info('Video start command accepted; creating WHIP/WebRTC connection')

    def _run_loop(self):
        self.event_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.event_loop)
        try:
            self.event_loop.run_until_complete(self._stream())
        except Exception as error:
            self.get_logger().error(f'WebRTC streaming task failed: {error}')
        finally:
            self.streaming.clear()
            self.event_loop.close()
            self.event_loop = None

    def _make_track(self):
        owner = self
        source = self.track_source
        video_frame_type = self.VideoFrame
        base_track = self.VideoStreamTrack

        class Track(base_track):
            async def recv(self):
                pts, time_base = await self.next_timestamp()
                frame = video_frame_type.from_ndarray(source.next_frame(), format='rgb24')
                frame.pts = pts
                frame.time_base = time_base
                return frame

        return Track()

    async def _stream(self):
        self.track_source = RosImageTrack(
            self,
            self.get_parameter('image_width').value,
            self.get_parameter('image_height').value,
        )
        self.image_subscription = self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.track_source.image_callback, 1)
        self.peer_connection = self.RTCPeerConnection()
        self.peer_connection.addTrack(self._make_track())
        # Preserve the ROS1 SRS interoperability behaviour: advertise a recv-only audio m-line.
        self.peer_connection.addTransceiver('audio', direction='recvonly')
        offer = await self.peer_connection.createOffer()
        await self.peer_connection.setLocalDescription(offer)
        await self._wait_for_ice_complete()
        headers = {'Content-Type': 'application/sdp'}
        # Keep the ROS1 SRS/WHIP SDP compatibility behaviour.  The platform
        # expects a complete audio m-line although this robot sends video only.
        offer_sdp = self._inject_srs_audio_section(self.peer_connection.localDescription.sdp)
        response = await asyncio.get_running_loop().run_in_executor(
            None,
            lambda: self.requests.post(
                self.whip_url(), data=offer_sdp,
                headers=headers, timeout=self.get_parameter('request_timeout_sec').value),
        )
        if response.status_code not in (200, 201):
            self.get_logger().error(f'WHIP server rejected stream: HTTP {response.status_code}: {response.text[:300]}')
            await self._cleanup()
            return
        await self.peer_connection.setRemoteDescription(
            self.RTCSessionDescription(sdp=response.text, type='answer'))
        self.get_logger().info('WHIP/WebRTC connection established; streaming video')
        while self.streaming.is_set() and not self.stop_requested.is_set() and rclpy.ok():
            if self.peer_connection.connectionState in ('failed', 'closed'):
                self.get_logger().warning(f'WebRTC connection ended: {self.peer_connection.connectionState}')
                break
            await asyncio.sleep(0.5)
        await self._cleanup()

    async def _wait_for_ice_complete(self):
        deadline = time.monotonic() + 5.0
        while self.peer_connection.iceGatheringState != 'complete' and time.monotonic() < deadline:
            await asyncio.sleep(0.1)

    @staticmethod
    def _inject_srs_audio_section(sdp: str) -> str:
        """Apply the original ROS1 fake-audio section required by SRS/WHIP.

        No audio device is opened and no audio is sent.  This only makes the
        SDP offer compatible with the platform's deployed video service.
        """
        def first(pattern, default=''):
            match = re.search(pattern, sdp)
            return match.group(1) if match else default

        cname = first(r'a=ssrc:\d+ cname:(\S+)', f'cname{random.randint(1000, 9999)}')
        audio_ssrc = random.randint(1, 2147483647)
        msid = f'audio{random.randint(1000, 9999)}'
        port = first(r'm=video (\d+)', '9')
        ice_ufrag = first(r'a=ice-ufrag:(\S+)')
        ice_pwd = first(r'a=ice-pwd:(\S+)')
        fingerprint = first(r'a=fingerprint:sha-256 (\S+)')
        setup = first(r'a=setup:(\S+)', 'actpass')
        candidates = re.findall(r'a=candidate:[^\r\n]+', sdp)
        audio_section = [
            f'm=audio {port} UDP/TLS/RTP/SAVPF 111',
            'c=IN IP4 0.0.0.0', 'a=rtcp:9 IN IP4 0.0.0.0', 'a=sendrecv',
            'a=extmap:1 urn:ietf:params:rtp-hdrext:sdes:mid', 'a=mid:1',
            f'a=msid:{msid} {msid}-audio', 'a=rtcp-mux',
            f'a=ssrc:{audio_ssrc} cname:{cname}',
            f'a=ssrc:{audio_ssrc} msid:{msid} {msid}-audio',
            'a=rtpmap:111 opus/48000/2', 'a=rtcp-fb:111 transport-cc',
            'a=fmtp:111 minptime=10;useinbandfec=1', *candidates,
            'a=end-of-candidates', f'a=ice-ufrag:{ice_ufrag}',
            f'a=ice-pwd:{ice_pwd}', f'a=fingerprint:sha-256 {fingerprint}',
            f'a=setup:{setup}',
        ]
        joiner = '\r\n' if '\r\n' in sdp else '\n'
        result, in_audio = [], False
        for line in sdp.split(joiner):
            if line.startswith('m=audio'):
                in_audio = True
                result.extend(audio_section)
            elif line.startswith('m='):
                in_audio = False
                result.append(line)
            elif not in_audio:
                result.append(line)
        return joiner.join(result)

    async def _cleanup(self):
        if self.image_subscription is not None:
            self.destroy_subscription(self.image_subscription)
            self.image_subscription = None
        self.track_source = None
        if self.peer_connection is not None:
            await self.peer_connection.close()
            self.peer_connection = None
        self.streaming.clear()

    def stop_streaming(self):
        if not self.streaming.is_set():
            return
        self.stop_requested.set()
        self.streaming.clear()
        if self.loop_thread and self.loop_thread.is_alive():
            self.loop_thread.join(timeout=5.0)
        self.get_logger().info('Video stream stopped')

    def destroy_node(self):
        self.stop_streaming()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebRTCStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
