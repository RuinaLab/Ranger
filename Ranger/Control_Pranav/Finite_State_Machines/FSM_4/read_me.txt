1, 2 May 2010, Pranav

FSM to try out for hip swing experiment.

Starts with hip, inner and outer feet in free state

On inner heelstrike is ready to do outer swing experiment
and on outer heelstrike is ready to do inner swing experiment.

parameters to tune

hip:
hip hold
ID_C_H_EH_H_ANG //stiffness
ID_C_H_EH_H_RATE //damping
ID_P_H_EH_H_TANG //target angle

hip swing
ID_A_H_PM_A0 //constant current

feet
stance
ID_C_F_ST_F_ANG //stiffness
ID_C_F_ST_F_RATE //damping
ID_P_F_ST_F_TANG //target angle

flipup
ID_C_F_FU_F_ANG //stiffness
ID_C_F_FU_F_RATE //damping
ID_P_F_FU_F_TANG //target angle

Also in state machine change
time_ehold2swing - time to wait to transition from hold state to swing state
time_swing - time to apply current ID_A_H_PM_A0 in millisecs