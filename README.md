# GAIN-SCHEDULED-IMPLICIT-AND-EXPLICIT-MPC-CONTROL-OF-MASS-SPRING-SYSTEM
GAIN SCHEDULED IMPLICIT AND EXPLICIT MPC CONTROL OF MASS-SPRING SYSTEM

This project mainly focuses on ‘Gain Scheduled Implicit and Explicit MPC Control of Mass-Spring
System. Here we used the method of gain scheduling, i.e. we made more than one linear
controller and activated those one at a time with the help of a scheduling signal to control a
non-linear plant. Two different MPC control techniques are used for this process named as
Implicit MPC Control and Explicit MPC control. Implicit MPC control is the traditional method
for the applications and is mainly used for the systems having large sampling times and for the
systems having larger computational times. Explicit MPC controllers are used for the
applications having lower computational times and for the systems having smaller
computational times. In this project, two Implicit MPC controllers and two explicit MPC
controllers have been generated for the two fold dynamics of the current Mass Spring system
problem. Pull Force is the system input and the position of mass M1 is the output. Hence with
the help of Simulink we are tracking position of mass M1 with a reference value R while using
two different techniques. Finally, the output of both the techniques is measured and compared
to know which is better technique in this project.
