from quanser.hardware import HIL, Clock
import array as arr 
import time
import signal
import math

import numpy as np

class Qube:
    def __init__(self,board_identifier,fs):
        
        self.card = HIL()
        self.card.open("qube_servo2_usb", str(board_identifier))
        self.fs = fs
        self.samples = HIL.INFINITE
        self.samples_in_buffer = math.ceil(0.1 * self.fs)
        self.clock = Clock.HARDWARE_CLOCK_0
        self.stop = True


        self.analog_channels = arr.array("i", [0])
        self.digital_channels = arr.array("i", [0])
        self.encoder_channels = arr.array("i", [0,1])
        self.other_channels = arr.array("i", [11000, 11001, 11002])    # LED red, green and blue

        self.num_analog_channels  = len(self.analog_channels)
        self.num_encoder_channels = len(self.encoder_channels)
        self.num_digital_channels = len(self.digital_channels)
        self.num_other_channels   = len(self.other_channels)

        self.voltages = arr.array("d", [0.0] * self.num_analog_channels)
        self.counts   = arr.array("i", [0] * self.num_encoder_channels)
        self.states   = arr.array("b", [1] * self.num_digital_channels)
        self.values   = arr.array("d", [0, 1, 0]) 

        

        self._reset_motor()
        self._reset_encoder()

        self._enable_motor()
        self._set_encoder_reader()

        print("This example controls the Quanser Qube Servo2 USB experiment at %g Hz." % self.fs)
        print("Press CTRL-C to stop the controller.\n")
        print ("Qube initialized...\n")




    def _reset_encoder(self):
        self.card.set_encoder_counts(self.encoder_channels, self.num_encoder_channels, self.counts)

    def _set_encoder_reader(self):
        self.encoder_reader_task = self.card.task_create_encoder_reader(self.samples_in_buffer,self.encoder_channels, self.num_encoder_channels)
        self.card.task_start(self.encoder_reader_task, self.clock, self.fs , self.samples)

    def _reset_motor(self):
        self.card.set_digital_directions(None, 0, self.digital_channels, self.num_digital_channels)
        self._write()
        self.card.write_analog(self.analog_channels, self.num_analog_channels, self.voltages)
    
    def _enable_motor(self):
        self.card.write_digital(self.digital_channels, self.num_digital_channels, self.states)
    
    def led(self, red = 0, green = 0, blue = 0):
        self.values   = arr.array("d", [red, green, blue]) 
        self._write()
      
    def _write(self):
        self.card.write(self.analog_channels, self.num_analog_channels, 
                    None, 0,
                    self.digital_channels, self.num_digital_channels,
                    self.other_channels, self.num_other_channels,
                    self.voltages, 
                    None, 
                    self.states, 
                    self.values)
    
    def motor(self, voltage):
        self.voltages[0] = voltage
        self._write_analog()

    def _write_analog(self):
        self.card.write_analog(self.analog_channels, self.num_analog_channels, self.voltages)
        
    def read_encoder(self):
        try:
            a = self.card.task_read_encoder(self.encoder_reader_task, 1, self.counts)
        except Exception as e:
            print(f"Failed to read encoder: {e}")
            raise
        return a
        
        # self.card.task_read_encoder(self.encoder_reader_task, 1, self.counts)
        # return self.counts
    
    def close(self):
        print("\nSignal handler...Closing...\n")
        self.stop = False
        self.card.task_stop(self.encoder_reader_task)
        self.card.task_delete(self.encoder_reader_task)
        
        self.voltages = arr.array("d", [0.0] * self.num_analog_channels)
        self.states   = arr.array("b", [0] * self.num_digital_channels)
    
        self.led(1,0,0)
        self._write_analog()
        self._write()
        self.card.close()
