from flash_controller.urbi_wrapper import UrbiWrapper




class FC:
    """ The FC class provides the configuration for the FLASH robot as provided by the _CONFIG_.u
        file.
    """


    def __init__(self):
        self.uw = UrbiWrapper()
        self.refresh()


    def refresh(self):
        """ Reads the current configuration from the robot and sets the variables of this 
            class accordingly. 
        """
        for var in FC.CONFIG_VARS:
            value = self.uw.send(var)[0]
            value = value.replace(b'true',  b'True')
            value = value.replace(b'false', b'False')
            setattr(self, var, eval(value))



    # List of joints with their respective settings
    # (http://flashrobotics.pl/hardware/emys/dimensions-and-joints)
    JOINTS_HEAD = [
        ['neck_pitch',      'robot.body.neck.pitch',                     -20.0,  30.0 ],
        ['head_yaw',        'robot.body.neck.head.yaw',                  -90.0,  90.0 ],
        ['head_pitch',      'robot.body.neck.head.pitch',                 -6.0,  29.0 ],
        ['disc_low',        'robot.body.neck.head.disc[down]',           -16.0,   5.0 ],
        ['disc_high',       'robot.body.neck.head.disc[up]',              -7.0,  15.0 ],
        ['left_eye_lid',    'robot.body.neck.head.eye[left].lid',          0.0, 120.0 ],
        ['left_eye_rot',    'robot.body.neck.head.eye[left].brow',       -45.0,  45.0 ],
        ['left_eye_trans',  'robot.body.neck.head.eye[left].trans',        0.0, 100.0 ],
        ['right_eye_lid',   'robot.body.neck.head.eye[right].lid',         0.0, 120.0 ],
        ['right_eye_rot',   'robot.body.neck.head.eye[right].brow',      -45.0,  45.0 ],
        ['right_eye_trans', 'robot.body.neck.head.eye[right].trans',       0.0, 100.0 ],
    ]

    # List of joints with their respective settings
    # (http://flashrobotics.pl/hardware/flash/dimensions-and-joints)
    JOINTS_LARM = [
        ['l_arm_q1',        'robot.body.arm[left][1]',                  -170.0, 170.0 ],
        ['l_arm_q2',        'robot.body.arm[left][2]',                   -90.0,  90.0 ],
        ['l_arm_q3',        'robot.body.arm[left][3]',                  -170.0, 170.0 ],
        ['l_arm_q4',        'robot.body.arm[left][4]',                   -50.0, 130.0 ],
        ['l_arm_q5',        'robot.body.arm[left][5]',                  -170.0, 170.0 ],
    ]

    # List of joints with their respective settings
    # (http://flashrobotics.pl/hardware/flash/dimensions-and-joints)
    JOINTS_RARM = [
        ['r_arm_q1',        'robot.body.arm[right][1]',                 -170.0, 170.0 ],
        ['r_arm_q2',        'robot.body.arm[right][2]',                  -90.0,  90.0 ],
        ['r_arm_q3',        'robot.body.arm[right][3]',                 -170.0, 170.0 ],
        ['r_arm_q4',        'robot.body.arm[right][4]',                  -50.0, 130.0 ],
        ['r_arm_q5',        'robot.body.arm[right][5]',                 -170.0, 170.0 ],
    ]

    # List of joints with their respective settings
    # (http://flashrobotics.pl/hardware/flash/dimensions-and-joints)
    JOINTS_LHAND = [
        ['l_hand_q1',       'robot.body.arm[left].hand.finger[1][1]',      0.0,  90.0 ],
        ['l_hand_q2',       'robot.body.arm[left].hand.finger[1][2]',      0.0,  90.0 ],
        ['l_hand_q3',       'robot.body.arm[left].hand.finger[1][3]',    -20.0,  20.0 ],
        ['l_hand_q4',       'robot.body.arm[left].hand.finger[2][1]',      0.0,  90.0 ],
        ['l_hand_q5',       'robot.body.arm[left].hand.finger[2][2]',      0.0,  90.0 ],
        ['l_hand_q6',       'robot.body.arm[left].hand.finger[2][3]',    -20.0,  20.0 ],
        ['l_hand_q7',       'robot.body.arm[left].hand.finger[3][1]',      0.0,  90.0 ],
        ['l_hand_q8',       'robot.body.arm[left].hand.finger[3][2]',      0.0,  90.0 ],
        ['l_hand_q9',       'robot.body.arm[left].hand.finger[3][3]',    -20.0,  90.0 ],
        ['l_hand_q10',      'robot.body.arm[left].hand.finger[4][1]',      0.0,  90.0 ],
        ['l_hand_q11',      'robot.body.arm[left].hand.finger[4][2]',    -20.0,  20.0 ],
        ['l_hand_q12',      'robot.body.arm[left].hand.finger[4][3]',      0.0,  90.0 ],
        ['l_arm_q6',        'robot.body.arm[left].hand.yaw',             -30.0,  30.0 ],
        ['l_arm_q7',        'robot.body.arm[left].hand.pitch',           -30.0,  30.0 ],
    ]

    # List of joints with their respective settings
    # (http://flashrobotics.pl/hardware/flash/dimensions-and-joints)
    JOINTS_RHAND = [
        ['r_hand_q1',       'robot.body.arm[right].hand.finger[1][1]',     0.0,  90.0 ],
        ['r_hand_q2',       'robot.body.arm[right].hand.finger[1][2]',     0.0,  90.0 ],
        ['r_hand_q3',       'robot.body.arm[right].hand.finger[1][3]',   -20.0,  20.0 ],
        ['r_hand_q4',       'robot.body.arm[right].hand.finger[2][1]',     0.0,  90.0 ],
        ['r_hand_q5',       'robot.body.arm[right].hand.finger[2][2]',     0.0,  90.0 ],
        ['r_hand_q6',       'robot.body.arm[right].hand.finger[2][3]',   -20.0,  20.0 ],
        ['r_hand_q7',       'robot.body.arm[right].hand.finger[3][1]',     0.0,  90.0 ],
        ['r_hand_q8',       'robot.body.arm[right].hand.finger[3][2]',     0.0,  90.0 ],
        ['r_hand_q9',       'robot.body.arm[right].hand.finger[3][3]',   -20.0,  90.0 ],
        ['r_hand_q10',      'robot.body.arm[right].hand.finger[4][1]',     0.0,  90.0 ],
        ['r_hand_q11',      'robot.body.arm[right].hand.finger[4][2]',   -20.0,  20.0 ],
        ['r_hand_q12',      'robot.body.arm[right].hand.finger[4][3]',     0.0,  90.0 ],
        ['r_arm_q6',        'robot.body.arm[right].hand.yaw',            -30.0,  30.0 ],
        ['r_arm_q7',        'robot.body.arm[right].hand.pitch',          -30.0,  30.0 ],
    ]
    
    JOINTS = [ JOINTS_HEAD, JOINTS_LARM, JOINTS_LHAND, JOINTS_RARM, JOINTS_RHAND ]
        
    # List of variables from the _CONFIG_.u file. 
    # (_CONFIG_.u)
    CONFIG_VARS = [
        '_RobotStructure',
        '_En_UPlayer',
        '_En_UPlayerNext',
        '_En_URecog',
        '_En_UKinectAudio',
        '_En_UTextToolAudio',
        '_En_UMP3',
        '_Speech_module',
        '_Speech_volume',
        '_Speech_param',
        '_Recog_module',
        '_Recog_param',
        '_Recog_input',
        '_Record_module',
        '_Player_module',
        '_PlayerNext_MasterVolume',
        '_PlayerNext_PlayerVolume',
        '_PlayerNext_MusicPlayerVolume',
        '_PlayerNext_MicRecordLevel',
        '_PlayerNext_KinectRecordLevel',
        '_En_UCamera',
        '_En_UKinectOpenNI',
        '_En_UKinectOpenNI2',
        '_En_UKinectVideo',
        '_En_UObjectDetector',
        '_En_UColorDetector',
        '_En_UMoveDetector',
        '_En_UFacet',
        '_En_UImageTool',
        '_En_UImageDisplay',
        '_Camera_flip',
        '_Camera_index',
        '_Kinect_Offset',
        '_Kinect_FaceTrackingEnable',
        '_Kinect_InteractionEnable',
        '_Kinect_DepthNearMode',
        '_Kinect_SkeletonTrackingMode',
        '_Kinect_SkeletonChooserMode',
        '_ObjectDetector_source',
        '_ObjectDetector_multi',
        '_Color1Detector_color',
        '_Color1Detector_source',
        '_Color2Detector_color',
        '_Color2Detector_source',
        '_Color3Detector_color',
        '_Color3Detector_source',
        '_Color4Detector_color',
        '_Color4Detector_source',
        '_MoveDetector_source',
        '_MoveDetector_frameBuffer',
        '_MoveDetector_duration',
        '_MoveDetector_smooth',
        '_FacetDetector_source',
        '_ImageDisplayWindows',
    
        # PLATFORM CONFIGURATION
        '_En_UPlayerNavigation',
        '_En_UAria',
        '_navigationPort',
        '_En_ARIA_Sonars',
        '_En_ARIA_Laser',
        '_ARIA_LaserType',
        '_ARIA_LaserStart',
        '_ARIA_LaserEnd',
        '_ARIA_Flipped',
        '_com_robot',
        '_com_laser',
        '_ARIA_LaserStartingBaud',
    
        # EMYS CONFIGURATION
        '_En_DynamixelHead',
        '_Dyn_FindServosHead',
        '_com_head',
        '_Head_IDs',
        '_Head_ZeroPosition',
        '_Head_DynamixelRatio',
        '_Head_HitecRatio',
        '_Head_TransRatio',
    
        # ARMS CONFIGURATION
        '_En_DynamixelArms',
        '_Dyn_FindServosArms',
        '_com_arms',
        '_Arms_IDs',
        '_Arms_ZeroPositions',
        '_Arms_Ratio',
        '_Arms_MaxSpeed',
    
        # HAND CONFIGURATION
        '_En_DynamixelHands',
        '_Dyn_FindServosHands',
        '_com_Lhands',
        '_com_Rhands',
        '_Hands_LIDs',
        '_Hands_RIDs',
        '_Hands_LZeroPosition',
        '_Hands_RZeroPosition',
        '_Hands_Finger_Ratio',
        '_Hands_Wrist_Ratio',
    
        # ML
        '_En_UKNearestML',
        '_En_UEigenfaces',
    
        # Network
        '_En_UBrowser',
        '_En_UMail',
        '_En_UFacebook',
        '_En_UGCalendar',
        '_En_UTextToolNetwork',
    
        # ... skipping some since they are not relevant right now ...
    
        # APPRAISAL
        '_En_UWordNet',
        '_En_USentiWordNet',
        '_En_UAnew',
        
        # EMOTIONS
        '_En_UPAD',
        '_En_UKNearestEmot',
        '_En_UWASABI',
        '_WASABI_RobotPersonality',   
    ]

