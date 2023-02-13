# import math

# distBtwWhe = 2.28
# distBtwWheR = distBtwWhe/2
# w_dia = 1.6
# w_r = w_dia/2
# pi = math.pi

# def inToRad(v):
#     v = v/w_r                # input=inches
#     if v > 6.28:             # max rotational velocity
#         print(f'ERROR: Invalid input ({v:.2f} > 6.28), setting it to 6.28')
#         v = 6.28
#     elif v < -6.28:
#         print(f'ERROR: Invalid input ({v:.2f} < -6.28), setting it to -6.28')
#         v = -6.28
#     return v

# t = 2.61
# s = pi*distBtwWheR / 2

# v = s/t

# print(inToRad(v))

# 1, 2, 3, 4
# n, e, s, w

# no 180 deg turns in this project
# def getRobotCommands(A):
#     cur_pose = "east"
#     commands = []
#     prev = []

#     for i in range(len(A)):
#         if i == 0:
#             prev = A[i]
#             continue
#         # horizontal movement
#         if prev[0] != A[i][0] and prev[1] == A[i][1]:
#             # left to right
#             if prev[0] < A[i][0]: # looking east 2
#                 if cur_pose == "north":
#                     commands.append(0)
#                 elif cur_pose == "south":
#                     commands.append(-1) 
#                 cur_pose = "east"          
            
#             # right to left
#             else: # looking west = 4, 
#                 if cur_pose == "north":
#                     commands.append(-1)
#                 elif cur_pose == "south":
#                     commands.append(0) 
#                 cur_pose = "west"          
            
#             commands.append(abs(prev[0] - A[i][0]))               
#         # vertical movement
#         elif prev[0] == A[i][0] and prev[1] != A[i][1]:
#             # going up
#             if prev[1] < A[i][1]:   
#                 if cur_pose == "east":
#                     commands.append(-1)
#                 elif cur_pose == "west":
#                     commands.append(0) 
#                 cur_pose = "north"          
#             # going down
#             else:                   
#                 if cur_pose == "east":
#                     commands.append(0)
#                 elif cur_pose == "west":
#                     commands.append(-1) 
#                 cur_pose = "south"          
            
#             commands.append(abs(prev[1] - A[i][1])) 
#         # 1/4 circle moves
#         elif prev[0] != A[i][0] and prev[1] != A[i][1]:
#             if cur_pose == "west":
#                 commands.append(-3)
#                 cur_pose = "south"
#             elif cur_pose == "east":
#                 commands.append(-2)
#                 cur_pose = "south"
#             elif cur_pose == "south":
#                 commands.append(-2)
#                 cur_pose = "west"
#         prev = A[i]

#     return commands


# # A = [[-5,-15],[15,-15],[15,15], [-10,15] ,[-15,10] ,[-15,5], [0,5] ,[5, 0] , [0,-5], [-10,-5] ,[-15,-1] ,[-15,-1]]
# A = [[-5,-15],[15,-15],[15,15], [-10,15] ,[-15,10] ,[-15,5], [0,5] ,[5, 0] , [0,-5], [-10,-5] ,[-15,-10] ,[-15,-15]]
# commands = [20, -1, 30, -1, 25, -3, 5, -1, 15, -2, -2, 10, -3, 5]
# print(getRobotCommands(A))

import math

# # goal -15, 10
# pose = [-10, 15]
# angle = math.pi/2
# R = 5
# time = 4.822344723260332
# x = R*math.sin(angle)
# y = R*(1+math.cos(angle))

# print(x, y)

s= 5*math.pi/2
R  =5
angle = s/R
x = R*math.cos(angle)
y = R*math.sin(angle)
# print(x, y)
