import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv(r"C:\Users\DSlobodzian\Desktop\Excel\DriveTrain.csv")
df.plot(x="timestamp", y = ["FR/LeftVoltage","FR/RightVoltage",])
df.plot(x="timestamp", y = ["FR/angle", "FR/vel"])
df.plot(x="timestamp", y = ["FR/LeftRPM","FR/RightRPM"])
df.plot(x="timestamp", y = ["FR/LeftCurrent","FR/RightCurrent"])
plt.show()
