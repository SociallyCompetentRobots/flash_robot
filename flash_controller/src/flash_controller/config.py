from flash_controller.urbi_wrapper import UrbiWrapper


# List of variables from the _CONFIG_.u file. Please consult this file for descriptions of the 
# configuration variables
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
        for var in CONFIG_VARS:
            value = self.uw.send(var)[0]
            value = value.replace(b'true',  b'True')
            value = value.replace(b'false', b'False')
            setattr(self, var, eval(value))
