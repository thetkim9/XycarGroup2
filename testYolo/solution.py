import time
import subprocess

st = time.time()
cmd = ['python3', '/home/user/testYolo/' + 'yolo.py', '-i', './data/dog.jpg']
output = subprocess.Popen(cmd, stdout=subprocess.PIPE).communicate()[0]
et = time.time()
print('yolo finished:', et - st)
print(output)
