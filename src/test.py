f = open("/home/robotica/catkin_ws/src/amazon_warehouse/src/mapa.txt", "r")
mapa = f.read()
with open("/home/robotica/catkin_ws/src/amazon_warehouse/src/mapa.txt", 'r') as f:
    l = []
    l = [line.split() for line in f]


for i, v in enumerate(l):
    for index, val in enumerate(v):
        if val == "1":
            print(type(i))