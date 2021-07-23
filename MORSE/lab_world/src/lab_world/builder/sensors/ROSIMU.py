from morse.builder.creator import SensorCreator

class Rosimu(SensorCreator):
    _classpath = "lab_world.sensors.ROSIMU.Rosimu"
    _blendname = "ROSIMU"

    def __init__(self, name=None):
        SensorCreator.__init__(self, name)

