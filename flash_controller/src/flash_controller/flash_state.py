import time

from flash_controller.urbi_wrapper  import UrbiWrapper
from flash_controller.config        import FC
from flash_controller.joint         import Joint


FS_USCRIPT1 = "function joints_val() {(%s);};"
FS_USCRIPT2 = "function joints_pos() {(%s);};"


class FlashState:


    def __init__(self):
        self.uw             = UrbiWrapper()
        self.fc             = FC()

        self.joints         = []
        self.joint_names    = {}
        self.joints_descr   = []
        self.zero_positions = []

        self.init()
        self.init_remote_scripts()


    def init(self):
        
        # get zero positions
        self.zero_positions.extend(self.fc._Head_ZeroPosition)
        self.zero_positions.extend(self.fc._Arms_ZeroPositions)
        self.zero_positions.extend(self.fc._Hands_LZeroPosition)
        self.zero_positions.extend(self.fc._Hands_RZeroPosition)

        # get joints description
        self.joints_descr.extend(self.fc.JOINTS_HEAD)
        self.joints_descr.extend(self.fc.JOINTS_LARM)
        self.joints_descr.extend(self.fc.JOINTS_RARM)
        self.joints_descr.extend(self.fc.JOINTS_LHAND)
        self.joints_descr.extend(self.fc.JOINTS_RHAND)

        # create joint objects
        for idx, j_descr in enumerate(self.joints_descr):
            joint = Joint(self.uw, j_descr[1], j_descr[2], j_descr[3], self.zero_positions[idx])
            setattr(self, j_descr[0], joint)
            self.joints.append(joint)


    def init_remote_scripts(self):

        # creates the joints_val function which returns the raw values for all joints in an tuple
        self.uw.send(FS_USCRIPT1 % ','.join([ '%s.val' % x.name for x in self.joints ]))

        # creates the joints_pos function which returns the positions for all joints in an tuple
        statements = ['(%s.val - %.4f) / %.4f' % (j.name, j.raw_zero, j.ratio) for j in self.joints]
        self.uw.send(FS_USCRIPT2 % ','.join(statements))
        

    def printAll(self):
        start = time.time()
        pos   = [ joint.pos for joint in self.joints ]

        print(time.time() - start)
        print(pos)


    def center(self):
        self.uw.send('robot.body.neck.head.BehaveNormal(2)')
        for joint in self.joints:
            print ('center', joint.name)
            joint.center()
            print (joint.pos, joint.raw)


    def joints_val(self):
        return eval(self.uw.send('joints_val')[0])


    def joints_pos(self):
        return eval(self.uw.send('joints_pos')[0])


if __name__ == '__main__':
    f = FlashState()
    f.printAll()
#     f.center()
    f.printAll()


    f.uw.send('robot.body.neck.head.BehaveNormal(2)')
    f.uw.send('robot.body.arm.MoveCenterDown2(2)')

    f.uw.send('robot.body.arm[right].MoveCenterUpper(3)')
    f.r_arm_q4.pos = 0
    f.r_arm_q5.pos = 60
    f.uw.send('robot.body.arm[right].hand.MoveOrientation(1,     0,   0)')
    f.uw.send('robot.body.arm[right].hand.MoveOrientation(0.9, -30, -10)')
    f.uw.send('robot.body.arm[right].hand.MoveOrientation(0.7, +30, -10)')
    f.uw.send('robot.body.arm[right].hand.MoveOrientation(0.7, -30, -10)')
    f.uw.send('robot.body.arm[right].hand.MoveOrientation(0.7, +30, -10)')
    f.uw.send('robot.body.arm[right].hand.MoveOrientation(0.7, -30, -10)')
    f.uw.send('robot.body.arm[right].hand.MoveOrientation(0.9, +30, -10)')
    f.uw.send('robot.body.arm[right].hand.MoveOrientation(1,     0,   0)')
    time.sleep(2)

    
    f.uw.send('robot.body.arm.MoveCenterDown2(3)')
    
