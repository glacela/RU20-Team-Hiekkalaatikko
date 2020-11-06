#!/usr/bin/env python

import cv2
import gi
import numpy as np
from multiprocessing import Lock

gi.require_version('Gst', '1.0')
from gi.repository import Gst

# IP AND PORT when using multicasting
# MULTICAST_IP = "224.1.1.1"
# MULTICAST_PORT = "5200"
STREAM_PORT = "5200"
IMAGE_HEIGHT = 1232
IMAGE_WIDTH = 1232


class GStreamerVideoSource():
    def __init__(self):
        Gst.init(None)

        self._width = IMAGE_WIDTH
        self._height = IMAGE_HEIGHT
        self._frame = None
        self._mutex = Lock()

        # See https://github.com/robot-uprising-hq/ai-video-streamer/blob/master/docs/Testing-AI-Video-Streamer.md
        # to see different ways to send the video stream from the
        # Raspberry Pi AI Video Streamer.

        # Video source when using multicasting
        # self.video_source = \
        #     f'udpsrc multicast-group={MULTICAST_IP} ' \
        #     f'auto-multicast=true port={MULTICAST_PORT}'

        # Video source when using direct streaming to one IP address
        self.video_source = \
            f'udpsrc port={STREAM_PORT}'
        self.video_codec = \
            '! application/x-rtp,encoding-name=JPEG,payload=26 ' \
            '! rtpjpegdepay ! jpegdec'
        self.video_decode = \
            '! decodebin ! videoconvert ' \
            '! video/x-raw,format=(string)BGR ! videoconvert'
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false drop=true'

        self.video_pipe = None
        self.video_sink = None

        self._run()

    def _start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]
        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def _gst_to_opencv(sample):
        """Transform byte array into np array
        Args:
            sample (TYPE): Description
        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps = sample.get_caps()
        array = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array

    def _crop_center(self, image, cropped_width, cropped_height):
        height, width, _ = image.shape
        startx = int((width - cropped_width) / 2)
        starty = int((height - cropped_height) / 2)

        stopx = startx + cropped_width
        stopy = starty + cropped_height
        image = image[starty:stopy, startx:stopx, :]
        return image

    def _resize(self, image, new_width, new_height):
        return cv2.resize(image, (new_width, new_height))

    def frame(self):
        """ Get Frame
        Returns:
            iterable: bool and image frame, cap.read() output
        """
        with self._mutex:
            new_frame = self._frame
        return new_frame

    def frame_available(self):
        """Check if frame is available
        Returns:
            bool: true if frame is available
        """
        with self._mutex:
            available = type(self._frame) != type(None)
        return available

    def _run(self):
        """ Get frame to update _frame
        """

        self._start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ])

        self.video_sink.connect('new-sample', self._callback)

    def _callback(self, sink):
        sample = sink.emit('pull-sample')
        new_frame = self._gst_to_opencv(sample)
        with self._mutex:
            self._frame = new_frame

        return Gst.FlowReturn.OK


if __name__ == '__main__':
    # Create the video object
    # Add port= if is necessary to use a different one
    video = GStreamerVideoSource()

    while True:
        # Wait for the next frame
        if not video.frame_available():
            continue

        frame = video.frame()
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
