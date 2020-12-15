# import numpy as np
# import time

# i = 0
# j = 1
# class test:
#     def __init__(self):
#         self.side = 0

#     def Fibonnaci(self):
#         global i, j
#         self.side = i + j
#         i = j
#         j = self.side

#     def search(self):
#         self.Fibonnaci()
#         if self.side % 4 == 0:
#             print("move up {}m".format(self.side))
#         elif self.side % 4 == 1:
#             print("move right {}m".format(self.side))
#         elif self.side % 4 == 2:
#             print("move down {}m".format(self.side))
#         elif self.side % 4 ==3:
#             print("move left {}m".format(self.side))

# while True:c
#     obj = test()
#     obj.search()
#     time.sleep(1)

counter = 0

for i in range(0 ,20):
    if(i%2 == 0):
        counter += 1
    print(counter)