import os

minv = float('inf')

for filename in os.listdir('data_t3'):
    if (".result" in filename):
        with open("data_t3/"+filename, 'r') as f:
            f.readline()
            r = f.readline().strip()
            if minv > float(r):
                minv = float(r)
                print(minv)
                print(filename)
