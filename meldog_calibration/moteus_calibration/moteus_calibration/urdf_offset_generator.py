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

from typing import List, Dict
from datetime import datetime

class UrdfOffsetGenerator:
  def __init__(self, filePath: str, jointNames: List[str], ids: List[int], type: str):
    self.filePath = filePath
    self.jointNames = jointNames
    self.ids = ids
    self.currentPositions = {}
    self.type = type

    if len(self.ids) != len(self.jointNames):
      raise RuntimeError("Joint name vector does not have same length as motor id vector!")
    
  def setPositions(self, currentPositions: Dict[int, float]):
    self.currentPositions = currentPositions

  def saveToFile(self):
    if len(self.currentPositions) != len(self.jointNames):
      raise RuntimeError("Position vector does not have same length as joint name vector!")
    with open(self.filePath, "w") as file:
      file.write("<?xml version=\"1.0\"?>\n")
      file.write("<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\">\n")
      file.write("\n")
      file.write(f"<!-- This file contains staring offsets for motors (meaning \"zero\" joint position) -->\n")
      file.write("<!-- Authors: Bartłomiej Krajewski (https://github.com/BartlomiejK2) -->\n")
      file.write(f"<!-- Generation time: {datetime.now().ctime()} -->\n")
      file.write("\n")
      for i in range(len(self.ids)):
        file.write(f"\t<!-- {self.jointNames[i]} offset -->\n")
        if self.type == "offset":
          file.write(f"  <xacro:property name=\"{self.jointNames[i]}_motor_offset\" value=\"{self.currentPositions[self.ids[i]]}\"/>\n")
        elif self.type == "maximum":
          file.write(f"  <xacro:property name=\"{self.jointNames[i]}_motor_maximum_position\" value=\"{self.currentPositions[self.ids[i]]}\"/>\n")
        elif self.type == "minimum":
          file.write(f"  <xacro:property name=\"{self.jointNames[i]}_motor_minimum_position\" value=\"{self.currentPositions[self.ids[i]]}\"/>\n")
        file.write("  \n")
      file.write("</robot>")


def main():
  jointNames = ["LFT", "LFH", "LFK", "RFT", "RFH", "RFK", 
                "LRT", "LRH", "LRK", "RRT", "RRH", "RRK"]
  
  ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
  
  positions = {name: 0.0 for name in jointNames}
  filePath = "motor_offsets.urdf.xacro"
  type = "offset"
  generator = UrdfOffsetGenerator(filePath, jointNames, ids, type)
  generator.setPositions(positions)
  generator.saveToFile()

if __name__ == "__main__":
  main()