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
from .terminal_interface import TerminalInterface
from .moteus_controller import MoteusController

MAIN_THREAD_FREQUENCY = 100
CONTROLLER_FREQUENCY = 100
TERMINAL_FREQUENCY = 10


async def main():
  parser = argparse.ArgumentParser(
                    prog='ros2 run moteus_calibration moteus_calibration',
                    description='Program to calibrate moteus motors, set starting ' \
                    'offsets for zero joint positions',
                    epilog='Authors: Bartłomiej Krajewski (https://github.com/BartlomiejK2)')
  
  parser.add_argument('-j', '--joints', nargs='+', help='List of joint names', required=True)
  parser.add_argument('-i', '--ids', nargs='+', help='List of motor ids', required=True)
  parser.add_argument('-o', '--output', help='File path for output', required=True)
  args = parser.parse_args()

  ids = []
  for id in args.ids:
    if id.isdigit():
      ids.append(int(id))
    else:
       raise ValueError("Motor ids should be integer values!")
  joints = args.joints
  filePath = args.output

  try:
    transport = moteus_pi3hat.Pi3HatRouter()
  except RuntimeError:
    print("Could not find pi3hat transport!")
    sys.exit()
  
  moteusController = MoteusController(ids, transport, CONTROLLER_FREQUENCY)
  terminalInterface = TerminalInterface(ids, joints, TERMINAL_FREQUENCY)
  urdfOffsetGenerator = UrdfOffsetGenerator(filePath, joints)

  async def moteusControllerTarget():
    await moteusController.run()

  def terminalInterfaceTarget():
    terminalInterface.run()

  terminalThread = threading.Thread(target=terminalInterfaceTarget())
  moteusThread = threading.Thread(target=moteusControllerTarget())

  terminalThread.start()
  moteusThread.start()

  sleepTime = 1 / MAIN_THREAD_FREQUENCY
  currentPositions = {}

  while True:
    escapeFlag = terminalInterface.getEscape()
    if escapeFlag:
      moteusController.setEscapeFlag(escapeFlag)
      break

    currentPositions = moteusController.getPositions()
    terminalInterface.setPositions(currentPositions)

    time.sleep(sleepTime)

  terminalThread.join()
  moteusController.join()

  urdfOffsetGenerator.setPositions(currentPositions)
  urdfOffsetGenerator.saveToFile()

if __name__ == "__main__":
  asyncio.run(main())