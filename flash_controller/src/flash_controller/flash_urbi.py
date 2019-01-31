U_FLASH_INIT = """ 
function stop() {
    robot.body.x.speed = 0;
    robot.body.yaw.speed = 0;    
    robot.body.arm.hand.MoveOrientation(2,0,0) &
    robot.body.arm.hand.MoveClose(4,2)         &
    robot.body.neck.head.BehaveNormal(2)       &
    robot.body.arm.MoveCenterDown(3);           
};
"""
