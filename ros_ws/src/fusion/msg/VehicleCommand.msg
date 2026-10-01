
#BO_ 225 VehicleControlRequest: 8 IntVehManGw_Flex
#SG_ BrakeHoldReq : 32|1@0+ (1,0) [0|0] "" Vector__XXX
#SG_ Brake_Rq : 27|9@0+ (0.05,-12.75) [-12.75|12.8] "m/sec\B2"  VehActControlGw_MABx,RemoteInterface
#SG_ Velocity_Rq : 23|12@0+ (0.01,0) [0|40.95] "m/s"  VehActControlGw_MABx,RemoteInterface
#SG_ StrgWhlAng_Rq : 7|16@0+ (0.03,-960) [-960|1006.05] "Deg"  VehActControlGw_MABx,RemoteInterface
#SG_ LongDir_Rq : 34|2@0+ (1,0) [0|3] "SED"  VehActControlGw_MABx,RemoteInterface

#LongDir_Rq - Gear/direction the automated driver is requesting the vehicle be in presently
#    Enum:
#    0. Neutral
#    1. Forward
#    2. Backward
#    3. Park
#StrgWhlAng_Rq - Desired vehicle path curvature, left is positive, right is negative. Correlates to steering wheel angle
#    Unit: Deg
#Velocity_Rq - Target longitudinal speed, always positive, closed loop speed controller uses brakes and propulsion to achieve and maintain speed
#    Unit: m/s
#Brake_Rq - Desired Friction Brake Request, arbitrated with output of speed controller (higher value wins)
#    Unit: m/s^2
#Brake Hold Request - Request brake system hold vehicle in place, disables speed controller
#    Boolean
 



uint8 long_dir_rq
float64 strgwhlang_rq
float64 velocity_rq
float64 brake_rq
bool brake_hold_rq