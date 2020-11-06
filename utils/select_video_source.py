import cv2


def select_video_source(selection):
    if selection == 'gstreamer':
        from utils.gstreamer_video_source import GStreamerVideoSource
        image_source = GStreamerVideoSource()

        def get_image():
            return image_source.frame()
        return get_image
    elif selection == 'webcam':
        cap = cv2.VideoCapture(0)

        def get_image():
            ret, frame = cap.read()
            return frame
        return get_image
    elif selection == 'ffmpeg':
        from utils.ffmpeg_video_source import FFMpegVideoSource
        image_source = FFMpegVideoSource()

        def get_image():
            return image_source.frame()
        return get_image
    else:
        raise Exception(f"Unknown video source, got: {selection}, "
                        f"but expected either 'gstreamer' or 'webcam'")
