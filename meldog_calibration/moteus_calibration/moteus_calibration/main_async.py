# Copyright (c) 2025, Koło Naukowe Robotyków
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.


# Authors: Bartłomiej Krajewski (https://github.com/BartlomiejK2)

import sys
import threading
import time
import argparse
import moteus_pi3hat
import asyncio

from .urdf_offset_generator import UrdfOffsetGenerator
from .terminal_interface_async import TerminalInterface
from .moteus_controller_async import MoteusController

MAIN_THREAD_FREQUENCY = 5
CONTROLLER_FREQUENCY = 100
TERMINAL_FREQUENCY = 5

async def main():
  parser = argparse.ArgumentParser(
                    prog='ros2 run moteus_calibration moteus_calibration',
                    description='Program to calibrate moteus motors, set starting ' \
                    'offsets for zero joint positions',
                    epilog='Authors: Bartłomiej Krajewski (https://github.com/BartlomiejK2)')
  
  parser.add_argument('-j', '--joints', nargs='+', help='List of joint names', required=True)
  parser.add_argument('-bi', '--bus_id', nargs='+', help='List of motor bus_id', required=True)
  parser.add_argument('-o', '--output', help='File path for output', required=True)
  parser.add_argument('-t', '--type', help='Type of data: [offset, maximum, minimum]', 
                      choices=['offset', 'maximum,' 'minimum'], required=True)
  args = parser.parse_args()

  busIdsDict = {}
  ids = []
  for bus_id in args.bus_id:
    _index = bus_id.find("_")
    busString = bus_id[:_index]
    idString = bus_id[_index + 1:]
    if idString.isdigit() and busString.isdigit():
      bus = int(busString)
      id = int(idString)
      if bus in busIdsDict:
        busIdsDict[bus].append(id)
      else:
        busIdsDict[bus] = [id]
      ids.append(id)
    else:
       raise ValueError("Motor ids should be integer values!")
  joints = args.joints
  filePath = args.output

  if len(joints) != len(ids):
    raise RuntimeError("Joint name vector does not have same length as motor id vector!")

  try:
    transport = moteus_pi3hat.Pi3HatRouter(servo_bus_map = busIdsDict)
  except RuntimeError:
    print("Could not find pi3hat transport!")
    sys.exit()
  
  moteusController = MoteusController(ids, transport, CONTROLLER_FREQUENCY)
  terminalInterface = TerminalInterface(ids, joints, TERMINAL_FREQUENCY)
  urdfOffsetGenerator = UrdfOffsetGenerator(filePath, joints, ids, args.type)

  sleepTime = 1 / MAIN_THREAD_FREQUENCY
  currentPositions = {}

  async def moteusControllerLoop():
    await moteusController.run()

  async def terminalInterfaceLoop():
    terminalInterface.run()

  async def mainLoop():
    while True:
      escapeFlag = terminalInterface.getEscape()
      if escapeFlag:
        moteusController.setEscapeFlag(escapeFlag)
        break

      currentPositions = moteusController.getPositions()
      terminalInterface.setPositions(currentPositions)
      lockedIds = terminalInterface.getLockedInPlace()
      moteusController.setLockedIds(lockedIds)

      await asyncio.sleep(sleepTime)

  
  moteusControllerTask = asyncio.create_task(moteusControllerLoop())
  terminalInterfaceTask = asyncio.create_task(terminalInterfaceLoop())
  mainTask = asyncio.create_task(mainLoop())

  await moteusControllerTask
  await terminalInterfaceTask
  await mainTask

  currentPositions = terminalInterface.getSavedPositons()

  urdfOffsetGenerator.setPositions(currentPositions)
  urdfOffsetGenerator.saveToFile()

if __name__ == "__main__":
  asyncio.run(main())