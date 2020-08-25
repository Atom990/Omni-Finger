import FTReading

FTSensor = FTReading.FTReading("192.168.1.1")
FTSensor.InitFT()
force = FTSensor.GetReading(1000)
print(force)
