import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq
    
signal = pd.read_csv('output.csv')

for channel in signal.columns[5:]:
    
    plt.figure()
    plt.plot((signal[channel]))
    plt.title(channel)
    plt.show()
    