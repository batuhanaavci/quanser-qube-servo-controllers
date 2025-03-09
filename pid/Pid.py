
import math
from scipy.signal import butter, lfilter
import matplotlib.pyplot as plt
import numpy as np

class Pid:
    def __init__(self, kp, ki, kd,fs=1000):
        # PID gains
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # PID variables initialization
        self.error = 0
        self.prev_error = 0
        self.integral = 0
        self.derivative = 0

        # Filter window, sampling frequency and period
        self.window = 200
        self.fs = fs
        self.period =1/fs

        self.derivatives = []
        self.filtered_derivatives = []
        self.errors = []
        self.outputs = []
        self.P_terms = []
        self.I_terms = []
        self.D_terms = []
        self.voltages = []
        self.commands = []
        self.positions = []
        print("PID controller initialized...\n")


    # Butterworth Lowpass Filter
    def _butter_lowpass(self, cutoff, fs, order=1):
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        return b, a
    
    # Butterworth Lowpass Filter
    def butter_lowpass_filter(self, data, cutoff, order=1):
        b, a = self._butter_lowpass(cutoff, self.fs, order=order)
        y = lfilter(b, a, data)
        return y
    
    # Update the PID controller
    def update(self, error):
     
        self.error = error
        self.integral += error * self.period
        self.derivative = (self.error - self.prev_error) / self.period
        self.prev_error = self.error

        #append the values to the lists
        P = self.kp * error
        I = self.ki * self.integral
        D = self.kd * self.derivative
        output = P + I + D

        self.errors.append(error)
        self.outputs.append(output)
        self.P_terms.append(P)
        self.I_terms.append(I)
        self.D_terms.append(D)

        return output
    
    # Update the PID controller with a filter, cutoff frequency and order
    def update_with_filter(self, error, cutoff, order=1):
        self.error = error
        self.integral += error * self.period
        self.derivative = (self.error - self.prev_error) / self.period
        self.prev_error = self.error

        self.derivatives.append(self.derivative)
        if len(self.derivatives) >= self.window:
            chunk = self.derivatives[-self.window:]
            filtered_derivative = self.butter_lowpass_filter(chunk, cutoff, order=order)[-1]
            self.filtered_derivatives.append(filtered_derivative)
        else:
            filtered_derivative = 0

        #append the values to the lists
        P = self.kp * error
        I = self.ki * self.integral
        D = self.kd * filtered_derivative
        output = P + I + D

        self.errors.append(error)
        self.outputs.append(output)
        self.P_terms.append(P)
        self.I_terms.append(I)
        self.D_terms.append(D)

        return output
    
    # Saturation function
    def saturation(self, threshold, input):
        u = input

        if input >= threshold:
            u = threshold
        elif input <= -threshold:
            u = -threshold
        
        return u
    
    def plot(self):

        time_vector = np.linspace(0, len(self.errors) * self.period, len(self.errors))


        plt.figure(figsize=(12, 6))
        plt.subplot(211)
        plt.plot(time_vector, self.positions, label='Position (Rad)')
        plt.plot(time_vector, self.commands, label='Command Signal', linestyle='--')
        plt.title('Postion and Command Signals Over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Rad')
        plt.legend()
        plt.grid(True)

        plt.subplot(212)
        plt.plot(time_vector, self.voltages, label='Voltage (V)')
        plt.title('Voltage Over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Voltage (V)')
        plt.legend()
        plt.grid(True)

        plt.tight_layout()
        plt.show()

        plt.figure(figsize=(10, 8))

        # Plot the error over time
        plt.subplot(321)
        plt.plot(time_vector, self.errors, label='Error')
        plt.title('Error over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Error')
        plt.grid(True)
        plt.legend()

        # Plot PID outputs over time
        plt.subplot(322)
        plt.plot(time_vector, self.outputs, label='PID Output (Without Saturation)')
        plt.title('PID Output over Time')
        plt.xlabel('Time (s)')
        plt.ylabel('Output')
        plt.grid(True)
        plt.legend()

        # Plot the individual components
        plt.subplot(323)
        plt.plot(time_vector, self.P_terms, label='Proportional (P)')
        plt.title('Proportional Component')
        plt.xlabel('Time (s)')
        plt.ylabel('P Term')
        plt.grid(True)
        plt.legend()

        plt.subplot(324)
        plt.plot(time_vector, self.I_terms, label='Integral (I)')
        plt.title('Integral Component')
        plt.xlabel('Time (s)')
        plt.ylabel('I Term')
        plt.grid(True)
        plt.legend()

        plt.subplot(325)
        plt.plot(time_vector, self.D_terms, label='Derivative (D)')
        plt.title('Derivative Component (Zoomed In)')
        plt.xlim(0, 0.1)
        plt.ylim(-5, 20)
        plt.xlabel('Time (s)')
        plt.ylabel('D Term')
        plt.grid(True)
        plt.legend()

        plt.subplot(326)
        plt.plot(time_vector[:len(self.derivatives)], self.derivatives, label='Derivative')
        
        if len(self.filtered_derivatives) > 0:
            # Create a time array that matches the length of the filtered_derivatives
            time_filtered = np.linspace(0, len(self.filtered_derivatives) * self.period, len(self.filtered_derivatives))
            plt.plot(time_filtered, self.filtered_derivatives, label='Filtered Derivative')
        
        plt.title('Derivative vs Filtered Derivative')
        plt.xlabel('Time (s)')
        plt.ylabel('Derivative')
        plt.grid(True)
        plt.legend()



        plt.tight_layout()
        plt.show()
