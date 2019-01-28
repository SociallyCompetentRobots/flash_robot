U_FLASH_INIT = """ 
function stop() {
    robot.body.x.speed = 0;
    robot.body.y.speed = 0;    
    robot.body.arm.hand.MoveOrientation(2,0,0) &
    robot.body.arm.hand.MoveClose(4,2)         &
    robot.body.neck.head.BehaveNormal(2)       &
    robot.body.arm.MoveSideDown(4);            
};
"""
