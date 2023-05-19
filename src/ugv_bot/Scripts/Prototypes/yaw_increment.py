from __future__ import division, print_function

# This script uses correct way to check for yaw angle incrments
# in both ACW direction (left Turn) and CW direction (right turn)

#Checks for every possible initial angle and turns left untill 90 is incremented

for i in range(0,360,10):               #i simulates every possible initial angle
  for j in range(0,180,10):             #j simulates the current angle while turning left
    f = (i + j)%360

    angular_disp = (f - i)%360          # this is the current angular displacement
    print("initial angle = ",i)
    print("current angle   =",f)
    if angular_disp <90:
      print("keep turning Nigga")
      print("-------------------")
    else :
      print("******************************")
      print("Well done bro !!!")
      print("Final Angle Displacement = ",angular_disp)
      break

    #print("angular displacement =",angular_disp)
  print("==================================")
  
'''
#For more specific angles turning 
#works for both CW and ACW turning
i = 89 
j = 359

f = (i + j)%360

if i>j:
  flag = 1
else: 
  flag = -1

angular_disp = (flag*(j - i))%360
print(angular_disp)
'''