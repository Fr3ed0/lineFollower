import os
import sys
import serial
import time
import signal
from gi.repository import Gtk

ser = serial.Serial("COM8", 9600)
# PORT = '/dev/ttyACM0'
# SPEED = 9600
class UI:
    def __init__(self):
        self.builder = Gtk.Builder()
        self.builder.add_from_file(os.path.join(os.getcwd(), 'data/arduino-led.ui'))
        self.window = self.builder.get_object('dialog1')
        self.aboutdialog = self.builder.get_object('aboutdialog1')
        self.bt_exit = self.builder.get_object('bt_exit')
        self.tbt_state = self.builder.get_object('tbt_state')
        self.imagemenuitem5 = self.builder.get_object('imagemenuitem5')
        self.imagemenuitem10 = self.builder.get_object('imagemenuitem10')

        self.window.connect('delete-event', self.quit)
        self.tbt_state.connect('toggled', self.on_button_toggled)
        self.bt_exit.connect('clicked', self.quit)
        self.imagemenuitem5.connect('activate', self.quit)
        self.imagemenuitem10.connect('activate', self.show_aboutdialog)
        self.window.show()

	def on_button_toggled(self, button):
		if button.get_active():
			state = ['1', 'on']
			button.set_label(state[1].upper())
			self.send_command(state[0])
		else:
			state = ['0', 'off']
			button.set_label(state[1].upper())
			self.send_command(state[0])
	
	def send_command(self, val):
		connection = serial.Serial( PORT, 
                                    SPEED,
                                    timeout=0,
                                    stopbits=serial.STOPBITS_TWO
                                    )
        connection.write(val)
        connection.close()