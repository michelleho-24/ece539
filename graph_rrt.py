import matplotlib.pyplot as plt

radius = 0.1
obs1 = plt.Circle((-0.117152, 0.0971121), radius, color='r')
obs2 = plt.Circle((-0.10171, 0.344291), radius, color='r')
obs3 = plt.Circle((0.0961944, 0.431613), radius, color='r')
obs4 = plt.Circle((-0.0290082, 0.433378), radius, color='r')
obs5 = plt.Circle((0.147471, 0.324343), radius, color='r')
obs6 = plt.Circle((0.136375, 0.216277), radius, color='r')
obs7 = plt.Circle((0.122134, 0.103535), radius, color='r')
obs8 = plt.Circle((-0.101076,  0.222749), radius, color='r')


fig, ax = plt.subplots()

ax.add_patch(obs1)
ax.add_patch(obs2)
ax.add_patch(obs3)
ax.add_patch(obs4)
ax.add_patch(obs5)
ax.add_patch(obs6)
ax.add_patch(obs7)
ax.add_patch(obs8)

# points 
x = []
y = []
count = 0
f = open('/Users/darrenchiu/Documents/SeniorFall/SCRS/ece539/Multi-Agent-Exploration/controllers/single_agent/rrt_sample.txt', 'r')

while True:
    count += 1
    
    line = f.readline()
    if not line:
        break

    x_num = float(line[0:11])
    x.append(x_num)

    index = line.index(",")
    y_sub = line[index+1:]

    index_2 = y_sub.index(",")
    y_sub2 = -float(y_sub[index_2+1:])

    y.append(y_sub2)

plt.scatter(x, y, c ="pink", s=5)

print(len(x))
print(x[0])
print(len(y))
print(y[0])


plt.xlim([-1,1])
plt.ylim([-1,2])

fig.savefig('plot4.png')