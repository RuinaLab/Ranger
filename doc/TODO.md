# TODO
Ideas for little projects to try on the road to making Ranger walk

After each project is completed, the results should be written-up in mark-down and saved in some reasonable place in the docs folder. The code for the solution should be tagged and then linked with the documentation.

## Estimator and Sensor Documentation:
- Add any important sensors to the page, and then clearly write all of the angle conventions. 

## Static Double Stance
- Write a simple PD controller that holds the robot with feet level and legs splayed. It should still work even if the robot is tipped up on one leg and then set back down. This is mostly to make sure that angle conventions and other basic assumptions are correct.

## Flying Tests:
 - Hang the robot from the ceiling and then program in a few slow periodic gaits. For example, make the hip angle track a sine curve in time. 
 - This would also be a good time to check the foot flip-up and flip-down code.
 - See if you can make the inner feet track the outer feet, and then move the outer foot angle around.
 - Rotate the outer legs, and see if the robot can keep the inner legs vertical.
