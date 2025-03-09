import signal
import math
import numpy as np

# Import the Qube class from Qube.py
from Qube import Qube
from LQR import LQR

# Function to define the pendulum's angle in the range of -pi to pi

def normalize_angle(full_angle):
    return (full_angle % (2 * np.pi)) - np.pi

def counts_to_radians(counts,is_alpha=False):
    if is_alpha:
        full_angle = counts * (2 * np.pi) / 2048
        return normalize_angle(full_angle)
    else:
        full_angle = counts * (-2 * np.pi) / 2048
        return full_angle
    
# This function returns square wave value at time 'time'
def input_signal(time, command_freq=0.1):
    if int(time // (1 / (2 * command_freq))) % 2 == 0:
        return math.pi/10
    else:
        return -math.pi/10


# Function to close the program
def close(signal,frame):
    global stop
    stop = False

stop = True

if __name__ == "__main__":

    # Sampling frequency 
    fs = 1000

    # LQR Gain Matrix
    # Here, this K is calculated for Q is identity matrix and R is 1 - standard K
    # Please refer to the gain_matrices.ipynb file for K value for different Q and R
    # You can change the K value by looking at the gain_matrices.ipynb file
    K = np.array([-1, 34.75077712, -1.49469806,  3.11102331])

    # Set the frequency and time for the input signal
    command_freq = 0.1
    time_t = 0

    # Initialize Qube
    qube = Qube(0,fs)

    # Initialize LQR Controller
    lqr = LQR(K,fs)

    # Initialize signal handler Ctrl + C
    signal.signal(signal.SIGINT, close)

    while stop:
        
        # Read the encoder values
        qube.read_encoder()    

        # Get the counts of the encoder
        theta = qube.counts[0]
        alpha = qube.counts[1]

        # Convert the counts to radians
        theta = counts_to_radians(theta,False)
        alpha = counts_to_radians(alpha,True)

        # Get the angular velocity
        theta_dot, alpha_dot = lqr.filter(theta,alpha)

        # Set the input signal (square wave)
        command = input_signal(time_t, command_freq)

        # State vector subtracted from reference (command)
        x = np.array([command-theta,0-alpha, 0-theta_dot, 0-alpha_dot])

        # LQR Control
        if np.abs(alpha) <= math.pi/12:  # +15 degree enable lqr
            u = lqr.balance_update(x)   # LQR control with -1 gain for counter-clockwise direction
            qube.led(0,1,0) # Green LED

        else:
            u=0 # Uncomment this line to disable swingup control, use your hand to swing up the pendulum
            #u = lqr.swingup_update(alpha, alpha_dot) # Swingup control, will automatically swing up the pendulum :)
            qube.led(0,0,1) # Blue LED

        # Saturation, don't let the voltage go above 5V or below -5V 
        # Please don't change this!
        voltage = lqr.saturation(5, u)

        # Increment the time
        time_t += 1/fs

        # Motor control
        qube.motor(voltage)
        
        # Append the values for plotting
        lqr.voltage_values.append(voltage)
    
    qube.close()

    lqr.plot()
