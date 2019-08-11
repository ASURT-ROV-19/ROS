import gi

gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst

class Gstreamer():
    def __init__(self, pipeline):
        Gst.init(None)
        self._pipeline = Gst.parse_launch(pipeline)
        self.running=False
    def start(self):
        try:
            ret = self._pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                raise Exception("Error starting the pipeline")
            self.running=True
            print("Start running",self.running)
        except :
            print("wasl el camera ya sa3eed")
            return

    def pause(self):
        self._pipeline.set_state(Gst.State.PAUSED)
        self.running = False
        print("Pause Camera ",self.running)
    def close(self):
        self.running = False
        self._pipeline.set_state(Gst.State.NULL)
#        print ("Stop running",self.running)

