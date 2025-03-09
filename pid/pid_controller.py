from Qube import Qube
from Pid import Pid
import math
import signal

def input_signal(time, command_freq=0.1):
    if int(time // (1 / (2 * command_freq))) % 2 == 0:
        return math.pi/4
    else:
        return -math.pi/4
    
def close(signal,frame):
    global stop
    stop = False

stop = True

if __name__ == "__main__":
    
    fs = 1000
    signal.signal(signal.SIGINT, close)

    # Initialize the qube
    qube = Qube(0, fs)

    # Create a pid controller object with Kp, Ki, Kd
    pid = Pid(1.3,0,0)

    # Set the initial properties of input signal
    command_freq = 0.1
    command = 0
    time_t = 0

    while stop:

        # Read the encoder
        qube.read_encoder()

        # Convert counts to radians
        position = qube.counts[0] * (2*math.pi) / 2048
        
        # Compute the input signal
        command = input_signal(time_t, command_freq)

        # Compute error
        error = command - position

        # Update the pid controller / or pid_with_filter

        #voltage = pid.update(error)
        #### OR #### Second argument is the cutoff frequency
        voltage = pid.update_with_filter(error, 20) 


        # Make sure voltage doesn't exceed threshold 10
        voltage = pid.saturation(threshold=10, input=voltage)

        # Apply the voltage to the motor
        qube.motor(voltage)

        # Store data for plotting
        pid.commands.append(command)
        pid.positions.append(position)
        pid.voltages.append(voltage)
        time_t += pid.period

    qube.close()
    pid.plot()