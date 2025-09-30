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

from typing import List

class UrdfOffsetParser:
  def __init__(self, filePath: str, jointNames: str):
   self.filePath = filePath
   self.jointNames = jointNames
   self.currentPositions = []
    
  def setPositions(self, currentPositions: List[float]):
    self.currentPositions = currentPositions

  def saveToFile(self):
    if(len(self.currentPositions) != len(self.jointNames)):
      raise RuntimeError("Position vector does not have same length as joint name vector!")
    with open(self.filePath, "w") as file:
      file.write("<?xml version=\"1.0\"?>\n")
      file.write("<robot xmlns:xacro=\"http://www.ros.org/wiki/xacro\">\n")
      for jointName, position in zip(self.jointNames, self.currentPositions):
        file.write(f"\t<!-- {jointName} offset -->\n")
        file.write(f"\t<xacro:property name=\"{jointName}_motor_offset\" value=\"{position}\"/>\n")
        file.write("\t\n")
      file.write("</robot>")


def main():
  jointNames = ["Amogus", "Sus", "A", "aAA"]
  positions = [0.1, 0.2, 0.3, 0.4]
  filePath = "test.urdf.xacro"
  parser = UrdfOffsetParser(filePath, jointNames)
  parser.setPositions(positions)
  parser.saveToFile()

if __name__ == "__main__":
    main()