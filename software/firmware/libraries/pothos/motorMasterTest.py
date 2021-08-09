import serial
from rover20_comms import rover20_comms as pothos

class Nodes():
    MotorNode1 = 1
    TowerNode = 2

class MotorNodeRegisters():
    speed = 0
    direction = 1
    temperature = 2
    current = 3


dataTypes = [['chr', 'chr', 'float', 'float'], ['chr', 'int', 'int', 'float']]


comms = pothos(len(dataTypes), dataTypes, 'COM25', 0.030, 500000)

comms.list_types()

print(comms.read(Nodes.MotorNode1,[MotorNodeRegisters.temperature]))
comms.write_single(Nodes.MotorNode1,MotorNodeRegisters.speed,chr(69))
comms.write_multiple(Nodes.MotorNode1,[MotorNodeRegisters.speed, MotorNodeRegisters.direction], [chr(69),chr(1)])
