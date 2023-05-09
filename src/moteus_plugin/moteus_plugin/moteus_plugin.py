#!/usr/bin/env python3
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget


class MoteusPlugin(Plugin):
    def __init__(self, context):
        super(MoteusPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MoteusPlugin')
        # Create QWidget
        self._widget = QWidget()
        ui_file = "src/moteus_plugin/moteus_plugin/moteus_plugin.ui"
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
