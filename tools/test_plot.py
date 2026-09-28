import matplotlib
import matplotlib.pyplot as plt
print('backend', matplotlib.get_backend())
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot([0,1,2],[0,1,0])
plt.show(block=False)
input('Plot should be visible — press Enter to exit\n')