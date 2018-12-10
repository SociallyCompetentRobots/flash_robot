# http://flashrobotics.pl/hardware/emys/dimensions-and-joints

from flash_controller.urbi_wrapper  import UrbiWrapper
from flash_controller.joint         import Joint
from flash_controller.sensor        import Sensor

class Head:
    """ The Flash class provides the Python API for the FLASH robot. """
    
    DEF_SPEED_ROTATION    = 10
    DEF_SPEED_TRANSLATION = 100

    MAX_SPEED_ROTATION    = 20
    MAX_SPEED_TRANSLATION = 200

    # [<attribute>, <name>, <pos_min>, <pos_max> ]
    JOINTS = [
        ['neck_pitch',      'robot.body.neck.pitch',                   -20.0,  30.0 ],
        ['head_yaw',        'robot.body.neck.head.yaw',                -90.0,  90.0 ],
        ['head_pitch',      'robot.body.neck.head.pitch',               -6.0,  29.0 ],
        ['disc_low',        'robot.body.neck.head.disc[down]',         -20.0,  30.0 ],
        ['disc_high',       'robot.body.neck.head.disc[up]',            -7.0,  15.0 ],
        ['left_eye_lid',    'robot.body.neck.head.eye[left].lid',        0.0, 120.0 ],
        ['left_eye_rot',    'robot.body.neck.head.eye[left].brow',     -45.0,  45.0 ],
        ['left_eye_trans',  'robot.body.neck.head.eye[left].trans',      0.0, 100.0 ],
        ['right_eye_lid',   'robot.body.neck.head.eye[right].lid',       0.0, 120.0 ],
        ['right_eye_rot',   'robot.body.neck.head.eye[right].brow',    -45.0,  45.0 ],
        ['right_eye_trans', 'robot.body.neck.head.eye[right].trans',     0.0, 100.0 ],
    ]

    
    def __init__(self):
        """ Creates an Flash API object and connects to the robot. """
        self.uw = UrbiWrapper()

        # check if the result from the dummy request fits; otherwise abort
        if not self.uw.isConnected:
            raise RuntimeError('Connection to Flash failed.')
        
        joint_zero_positions = eval(self.uw.send('_Head_ZeroPosition')[0])

        # create joints
        for idx, info in enumerate(Head.JOINTS):            
            joint = Joint(self.uw, info[1], info[2], info[3], joint_zero_positions[idx])
            setattr(self, info[0], joint)

        # center joints
        for joint in Head.JOINTS:
            getattr(self, joint[0]).center()        

        # create sensors
        self.touch_up       = Sensor(self.uw, 'robot.body.neck.head.sensor[up]',       0, 2000)
        self.touch_left     = Sensor(self.uw, 'robot.body.neck.head.sensor[left]',     0, 2000)
        self.touch_front    = Sensor(self.uw, 'robot.body.neck.head.sensor[front]',    0, 2000)
        self.touch_right    = Sensor(self.uw, 'robot.body.neck.head.sensor[right]',    0, 2000)
        self.touch_down     = Sensor(self.uw, 'robot.body.neck.head.sensor[down]',     0, 2000)


    def getStatusStr(self):

        status  = ["Head Status"]
        status.append("===========")
        status.append('- Joints')
        status.append(str(self.neck_pitch))
        status.append(str(self.head_yaw))
        status.append(str(self.head_pitch))
        status.append(str(self.disc_low))
        status.append(str(self.disc_high))
        status.append(str(self.left_eye_lid))
        status.append(str(self.left_eye_rot))
        status.append(str(self.left_eye_trans))
        status.append(str(self.right_eye_lid))
        status.append(str(self.right_eye_rot))
        status.append(str(self.right_eye_trans))

        status.append('- Sensors')
        status.append(str(self.touch_up))
        status.append(str(self.touch_left))
        status.append(str(self.touch_front))
        status.append(str(self.touch_right))
        status.append(str(self.touch_down))

        return '\n'.join(status)
