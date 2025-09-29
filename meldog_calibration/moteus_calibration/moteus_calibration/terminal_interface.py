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


from threading import Lock
from copy import deepcopy
from typing import List
import time

from rich.console import Console
from rich.table import Table
from rich.panel import Panel
from rich.live import Live
from rich.layout import Layout
from rich.align import Align

from datetime import datetime

import sys
import select
import tty
import termios

# Class for controlling moteuses via terminal
class TerminalInterface:

  def __init__(self, ids: List[int], jointNames: List[str], frequency: float = 10):
    self.ids = ids
    self.jointNames = jointNames
    self.currentPositions = {id: 0.0 for id in self.ids}
    self.lockedInPlace = {id: False for id in self.ids}
    self.sleepTime = 1 / frequency
    self.userInput = ""
    self.escape = False

    self.oldConsolSettings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    self.lock = Lock()

    self.console = Console()
    self.layout = self.generateLayout()
    self.layout["header"].update(self.generateHeader())
    self.layout["main"].update(self.generateTable())
    self.layout["commandline"].update(self.generateCommandLine())

  def __del__(self):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.oldConsolSettings)

  def run(self):
    with Live(self.layout, refresh_per_second=10, screen=True) as live:
      while True:
        time.sleep(self.sleepTime)
        self.layout["header"].update(self.generateHeader())
        self.layout["main"].update(self.generateTable())
        self.layout["commandline"].update(self.generateCommandLine())
        live.refresh()
      
  
  def generateTable(self):
    table = Table(title= "[b][magenta]Joint Data[/b]")
    table.add_column("[b]Joint name[/b]", justify="center", style="magenta", no_wrap=True)
    table.add_column("[b]Motor ID[/b]", justify="center", style="magenta")
    table.add_column("[b]Current Position \[rad][/b]", justify="center", style="magenta")
    table.add_column("[b]Locked in place?[/b]", justify="center", style="magenta")
    with self.lock:
      for name, id, position, locked in zip(self.jointNames, self.ids, self.currentPositions.values(), 
                                    self.lockedInPlace.values()):
        if(locked):
          state = "[b][green]True[/b]"
        else:
          state = "[b][red]False[/b]"
        table.add_row(f"[b]{name}[/b]", f"[b]{id}[/b]", f"[b]{position:.2f}[/b]", state)
    table = Align.center(table, vertical="top")
    return Panel(table, border_style="magenta")

  def generateHeader(self):
    grid = Table.grid(expand=True)
    grid.add_column(justify="center", ratio=1)
    grid.add_row(
		  "[b][blue]Meldog Calibration Tool[/b]")
    return Panel(grid)
  
  def generateCommandLine(self):
    text = "Lock or unlock moteus via id number and press ENTER or " \
    "press ESC to save and exit:\n"
    self.userInput += self.readUserChar()
    if len(self.userInput) > 0 and self.userInput[-1] == '\x1b':
      self.escape = True
    if len(self.userInput) > 0 and self.userInput[-1] == "\n":
      self.userInput = self.userInput.strip('\n')
      if self.userInput.isdigit():
        try:
          self.lockedInPlace[int(self.userInput)] = not self.lockedInPlace[int(self.userInput)]
        except KeyError:
          pass
      self.userInput = ""
    else:
      text = text + self.userInput
    return Panel(renderable=text, title="Command Line Interface")
  
  def generateLayout(self):
    layout = Layout(name="root")
    layout.split(
      Layout(name="header", size=3), 
      Layout(name="main", size=25),
      Layout(name="commandline"))
    return layout
  
  def setPositions(self, currentPositions):
    with self.lock:
      self.currentPositions = currentPositions
  
  def getLockedInPlace(self):
    with self.lock:
      lockedInPlace =  deepcopy(self.lockedInPlace)
    return lockedInPlace
  
  def readUserChar(self):
    output = ""
    if(select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])):
        output = sys.stdin.read(1)
    return output
  
  def getEscape(self):
    return self.escape


def main():
  jointNames = ["Amogus", "Sus", "A", "aAA"]
  ids = [1, 2, 3, 4]
  terminalInterface = TerminalInterface(ids, jointNames)
  terminalInterface.run()

if __name__ == "__main__":
    main()