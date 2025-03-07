import asyncio
import math
import moteus
import moteus.multiplex
import numpy as np
import time
import sys
from enum import Enum, IntEnum
import pandas as pd

class TestType(Enum):
  SOUND = "sound"
  DYNAMIC_TORQUE = "dynamic_torque"
  STATIC_TORQUE = "static_torque"
  PID = "pid"

class DiagnosticType(IntEnum):
  REFERENCE_POSITION = moteus.Register.CONTROL_POSITION
  POSITION = moteus.Register.POSITION
  VELOCITY = moteus.Register.VELOCITY
  TORQUE = moteus.Register.TORQUE
  TEMPERATURE = moteus.Register.TEMPERATURE
  MOTOR_TEMPERATURE = moteus.Register.MOTOR_TEMPERATURE
  POWER = moteus.Register.POWER
  VOLTAGE = moteus.Register.VOLTAGE
  FAULT = moteus.Register.FAULT
  Q_CURRENT = moteus.Register.Q_CURRENT
  D_CURRENT = moteus.Register.D_CURRENT


def mapDiagnosticStringToEnum(diagnosticString: str) -> DiagnosticType:
  if(diagnosticString == "reference_position"):
    return DiagnosticType.REFERENCE_POSITION
   
  elif(diagnosticString == "position"):
    return DiagnosticType.POSITION
   
  elif(diagnosticString == "velocity"):
    return DiagnosticType.VELOCITY
   
  elif(diagnosticString == "torque"):
    return DiagnosticType.TORQUE
   
  elif(diagnosticString == "temperature"):
    return DiagnosticType.TEMPERATURE
  
  elif(diagnosticString == "motor_temperature"):
    return DiagnosticType.MOTOR_TEMPERATURE
   
  elif(diagnosticString == "power"):
    return DiagnosticType.POWER
   
  elif(diagnosticString == "voltage"):
    return DiagnosticType.VOLTAGE
   
  elif(diagnosticString == "fault"):
    return DiagnosticType.FAULT
  
  elif(diagnosticString == "q_current"):
    return DiagnosticType.Q_CURRENT

  elif(diagnosticString == "d_current"):
    return DiagnosticType.D_CURRENT
   
  return DiagnosticType.POSITION


class TestParameters:
    def __init__(self):
      # Default parameters
      self.totalTime = 10.0  # [s]
      self.frequency = 100.0 # [Hz]
      self.type = TestType.DYNAMIC_TORQUE
      self.diagnosticTypes = [DiagnosticType.POSITION, 
                               DiagnosticType.TORQUE,
                               DiagnosticType.TEMPERATURE,
                               DiagnosticType.POWER]
      self.diagnosticNames = ["position",
                                "torque",
                                "temperature",
                                "power"]
      
      self.outputPath = "./output_file.csv"
      self.exitFlag = False
      
class MotorParameters:
  def __init__(self):
      # Default parameters
      self.motorId = 0
      self.maxVelocity = 10   # [rad/s]
      self.maxTorque = 0.2    # [Nm]


def argparser(arguments: list[str]):
  testParams = TestParameters()
  motorParams = MotorParameters()
  argvIndex = 1
  while argvIndex < len(arguments):
    if(arguments[argvIndex] == "--help"):
        print("Moteus Test Script")
        print("Command Line Arguments:")
        print("--id                 Moteus ID")
        print("--max_velocity       Maximum velocity [rad/s]")
        print("--max_torque         Maximum torque [Nm]")
        print("--frequency          Frequency of script control loop")
        print("--test_time          Total time of test")
        print("--output_path        Output results to given file")
        print("--type               Type of the test: sound, dynamic_torque, static_torque, pid")
        print("--diagnostic_values  All diagnostic values to control: ")
        print("reference_position, position, velocity, torque, temperature")
        print("power, voltage, fault, q_current, d_current, motor_temperature")
        print("")
        print("Example command:")
        print("python3 moteus_tests.py --type sound --diagnostic_values position temperature voltage")
        testParams.exitFlag = True

    if(arguments[argvIndex] == "--id"):
        motorParams.motorId = int(arguments[argvIndex + 1])
        argvIndex += 1
        continue
    
    if(arguments[argvIndex] == "--max_velocity"):
        motorParams.maxVelocity= float(arguments[argvIndex + 1]) / (2 * math.pi) # rad/s -> rot/s
        argvIndex += 1
        continue
    
    if(arguments[argvIndex] == "--max_torque"):
        motorParams.maxTorque = float(arguments[argvIndex + 1])
        argvIndex += 1
        continue

    if(arguments[argvIndex] == "--output_path"):
        testParams.outputPath = str(arguments[argvIndex + 1])
        argvIndex += 1
        continue
      
    if(arguments[argvIndex] == "--frequency"):
        testParams.frequency = float(arguments[argvIndex + 1])
        argvIndex += 1
        continue
      
    if(arguments[argvIndex] == "--test_time"):
        testParams.totalTime = float(arguments[argvIndex + 1])
        argvIndex += 1
        continue
      
    if(arguments[argvIndex] == "--type"):
        testParams.type = TestType(arguments[argvIndex + 1])
        argvIndex += 1
        continue
      
    if(arguments[argvIndex] == "--diagnostic_values"):
        while (((argvIndex + 1) < len(arguments)) or ("--" in arguments[argvIndex + 1])):
          diagnosticValue = mapDiagnosticStringToEnum(str(arguments[argvIndex + 1]))
          testParams.diagnosticTypes.append(diagnosticValue)
          testParams.diagnosticNames.append(str(arguments[argvIndex + 1]))
          argvIndex += 1
        continue
      
    argvIndex += 1

  return testParams, motorParams


async def soundTest(controller: moteus.Controller,
                    diagnosticNames: list[str],
                    diagnosticDict: dict[DiagnosticType, np.ndarray],
                    motorParams: MotorParameters,
                    startState, maxIter: int,
                    frequency: float):
  
  print("Starting Sound Test!")

  maxTorque = motorParams.maxTorque
  maxVelocity = motorParams.maxVelocity

  oneSecondInIter = frequency

  oneIterTime = 1 / frequency
  oneEpisodeIter = maxIter / 3

  velocity = maxVelocity / 3

  state = startState

  startTime = time.perf_counter()

  i = 0
  while i < maxIter:
    if(i == 0):
      velocity = maxVelocity / 3
      print(f"First episode: {velocity} rad/s ")
      startPos = state.values[moteus.Register.POSITION]
      startTime = time.perf_counter()

    elif(i == oneEpisodeIter):
      velocity = -2 * maxVelocity / 3
      print("New episode in 10 seconds..")
      await asyncio.sleep(10)
      print(f"Second episode: {velocity} rad/s ")
      startPos = state.values[moteus.Register.POSITION]
      startTime = time.perf_counter()
    
    elif(i == 2 * oneEpisodeIter):
      velocity = maxVelocity 
      print("New episode in 10 seconds..")
      await asyncio.sleep(10)
      print(f"Third episode: {velocity} rad/s ")
      startPos = state.values[moteus.Register.POSITION]
      startTime = time.perf_counter()

    current_time = time.perf_counter()
    dt = current_time - startTime
    current_position = (velocity * dt) + startPos
    state = await controller.set_position(position = current_position, maximum_torque = maxTorque, velocity_limit = velocity, query=True)
    
    for diagnosticType, diagnosticArray in diagnosticDict.items():
      diagnosticArray[i] = state.value[diagnosticType]

    if(i // oneSecondInIter == 0):
      diagnosticMessage = "Current State: "
      for diagnosticName, diagnosticValue in zip(diagnosticNames, diagnosticDict.values()):
        diagnosticMessage += f"{diagnosticName} = {diagnosticValue[i]:.2f} "
      print(diagnosticMessage, end="\r", flush=True)

    while(time.perf_counter() - current_time < oneIterTime):
      pass

    i += 1

async def testMain(testParams: TestParameters ,motorParams: MotorParameters) -> None:
  totalTime = testParams.totalTime
  frequency = testParams.frequency
  type = testParams.type
  diagnosticTypes = testParams.diagnosticTypes
  diagnosticNames = testParams.diagnosticNames
  outputPath = testParams.outputPath

  motorId = motorParams.motorId

  maxIter = int(totalTime*frequency)

  diagnosticDict = {}

  for diagnosticValue in diagnosticTypes:
     diagnosticDict[diagnosticValue] = np.empty(maxIter)
  

  try:
      transport = moteus.Fdcanusb()
  except RuntimeError:
      print("No moteus detected")
      sys.exit()

  queryResolution = moteus.QueryResolution()
  if DiagnosticType.MOTOR_TEMPERATURE in diagnosticTypes:
    queryResolution.motor_temperature = moteus.multiplex.INT8
  
  if DiagnosticType.POWER in diagnosticTypes:
    queryResolution.power = moteus.multiplex.F32
  
  if DiagnosticType.Q_CURRENT in diagnosticTypes:
    queryResolution.q_current = moteus.multiplex.F32

  if DiagnosticType.D_CURRENT in diagnosticTypes:  
    queryResolution.d_current = moteus.multiplex.F32

  controller = moteus.Controller(id = motorId, query_resolution = queryResolution, transport = transport)

  print("Stopping moteus!")
  await controller.set_stop()

  startState = await controller.set_position(position=math.nan, query=True)

  if(type == TestType.SOUND):
    await soundTest(controller,
                          diagnosticNames,
                          diagnosticDict,
                          motorParams,
                          startState,
                          maxIter,
                          frequency)
    
  await controller.set_stop()
  dataframe = pd.DataFrame()
  
  for diagnosticName, diagnosticType in zip(diagnosticNames, diagnosticDict.keys()):
     dataframe[diagnosticName] = diagnosticDict[diagnosticType].tolist()

  dataframe.to_csv(outputPath, index=False)

def main():
  print("Starting test!")
  testParams, motorParams = argparser(sys.argv)
  if(testParams.exitFlag): sys.exit()
  asyncio.run(testMain(testParams, motorParams))
  print("Test finished!")

main()