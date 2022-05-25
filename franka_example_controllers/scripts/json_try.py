import json

while True:
    print("hi")

f = open("traj1.json", "r").read()

a = json.loads(f)["points"][0]

print(a[0])