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
 

import moteus
from threading import Lock
from copy import deepcopy
from asyncio import sleep
from typing import List, Dict
from math import pi

# Class to control moteuses for calibration purposes
class MoteusController:
  def __init__(self, ids: List[int], transport: moteus.Transport, frequency: float):
    self.ids = ids
    self.transport = transport
    self.sleepTime = 1 / frequency
    self.currentPositions = {id: 0.0 for id in self.ids}
    self.lockedIds = {id: False for id in self.ids}
    
    self.positionLock = Lock()
    self.escapeFlagLock = Lock()
    self.idLock = Lock()

    self.escapeFlag = False
    
    # Connect to moteuses:
    self.servos = {id: moteus.Controller(id=id, transport=self.transport) 
                   for id in self.ids}
  
  # Get current positions in thread-safe manner
  def getPositions(self):
      with self.positionLock:
          positions = deepcopy(self.currentPositions)
      return positions
  
  def setLockedIds(self, ids):
    with self.idLock:
      self.lockedIds = ids
  
  # Spawn or despawn moteuses
  async def spawn(self):
    commands = [servo.make_stop(query = True) for servo in self.servos.values()]
    return await self.transport.cycle(commands)
  
  # Query moteuses
  async def query(self):
    commands = [servo.make_query() for servo in self.servos.values()]
    return await self.transport.cycle(commands)
  
  # Set escape flag
  def setEscapeFlag(self, flag):
    with self.positionLock:
      self.escapeFlag = flag

  # Query or lock moteuses
  async def queryOrLock(self):
    commands = []
    with self.idLock:
      for id in self.ids:
        if self.lockedIds[id]:
          command = self.servos[id].make_position(position= self.currentPositions[id], 
                                            maximum_torque = 0.2, 
                                            query=True)
        else:
          command = self.servos[id].make_query()
        commands.append(command)
    return await self.transport.cycle(commands)
       
  # Main funtion
  async def run(self):
    # Restart moteuses:
    results = await self.spawn()
    with self.positionLock:
        for result in results:
          self.currentPositions[result.id] = result.values[moteus.Register.POSITION] * 2 * pi

    # Main control loop:
    while True:   
      with self.escapeFlagLock:
        if(self.escapeFlag):
          await self.spawn()
          return
        
      results = await self.queryOrLock()

      with self.positionLock:
        for result in results:
          self.currentPositions[result.id] = result.values[moteus.Register.POSITION] / (2 * pi)
      await sleep(self.sleepTime)