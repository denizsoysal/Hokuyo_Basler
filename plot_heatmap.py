import pandas as pd
import matplotlib.pyplot as plt
from IPython.display import display, HTML

name_file = "inside_wall_dry\RecordingBeforeSpraying_14_15_54\depth_values_real_sense_20"
path_to_csv= "inside_wall_dry\RecordingBeforeSpraying_14_15_54\depth_values_real_sense_20" + ".csv"
df= pd.read_csv(path_to_csv)
df.columns[0].split(',')
display(df)

plt.imshow(df,cmap='hot',interpolation='nearest', vmin=1, vmax=2)
plt.savefig(name_file)

plt.show()

