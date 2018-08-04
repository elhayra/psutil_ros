#Return hardware temperatures. Each entry is a named tuple representing a certain hardware #temperature sensor (it may be a CPU, an hard disk or something else, depending on the OS and its #configuration). All temperatures are expressed in celsius unless fahrenheit is set to True. If #sensors are not supported by the OS an empty dict is returned.

SensorTemp[] temps
