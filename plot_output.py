import pandas as pd
import matplotlib.pyplot as plt
    
signal = pd.read_csv('output.csv')

for channel in signal.columns[1:]:
    
    plt.figure()
    plt.plot(signal[channel])
    plt.title(channel)
    plt.show()
    