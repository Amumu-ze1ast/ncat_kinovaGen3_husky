
"use strict";

let ApiOptions = require('./ApiOptions.js');
let KortexError = require('./KortexError.js');
let ErrorCodes = require('./ErrorCodes.js');
let SubErrorCodes = require('./SubErrorCodes.js');
let CoggingFeedforwardModeInformation = require('./CoggingFeedforwardModeInformation.js');
let ActuatorConfig_SafetyLimitType = require('./ActuatorConfig_SafetyLimitType.js');
let VectorDriveParameters = require('./VectorDriveParameters.js');
let ActuatorConfig_ServiceVersion = require('./ActuatorConfig_ServiceVersion.js');
let TorqueOffset = require('./TorqueOffset.js');
let Servoing = require('./Servoing.js');
let CommandMode = require('./CommandMode.js');
let CustomDataSelection = require('./CustomDataSelection.js');
let CommandModeInformation = require('./CommandModeInformation.js');
let StepResponse = require('./StepResponse.js');
let ActuatorConfig_ControlModeInformation = require('./ActuatorConfig_ControlModeInformation.js');
let RampResponse = require('./RampResponse.js');
let AxisOffsets = require('./AxisOffsets.js');
let TorqueCalibration = require('./TorqueCalibration.js');
let EncoderDerivativeParameters = require('./EncoderDerivativeParameters.js');
let CustomDataIndex = require('./CustomDataIndex.js');
let PositionCommand = require('./PositionCommand.js');
let LoopSelection = require('./LoopSelection.js');
let SafetyIdentifierBankA = require('./SafetyIdentifierBankA.js');
let ControlLoopParameters = require('./ControlLoopParameters.js');
let ControlLoopSelection = require('./ControlLoopSelection.js');
let CoggingFeedforwardMode = require('./CoggingFeedforwardMode.js');
let ControlLoop = require('./ControlLoop.js');
let FrequencyResponse = require('./FrequencyResponse.js');
let AxisPosition = require('./AxisPosition.js');
let ActuatorConfig_ControlMode = require('./ActuatorConfig_ControlMode.js');
let ActuatorCyclic_MessageId = require('./ActuatorCyclic_MessageId.js');
let ActuatorCyclic_CustomData = require('./ActuatorCyclic_CustomData.js');
let CommandFlags = require('./CommandFlags.js');
let ActuatorCyclic_Feedback = require('./ActuatorCyclic_Feedback.js');
let StatusFlags = require('./StatusFlags.js');
let ActuatorCyclic_ServiceVersion = require('./ActuatorCyclic_ServiceVersion.js');
let ActuatorCyclic_Command = require('./ActuatorCyclic_Command.js');
let ActionNotificationList = require('./ActionNotificationList.js');
let GpioEvent = require('./GpioEvent.js');
let OperatingModeNotification = require('./OperatingModeNotification.js');
let ControllerConfigurationMode = require('./ControllerConfigurationMode.js');
let JointTrajectoryConstraintType = require('./JointTrajectoryConstraintType.js');
let JointTorques = require('./JointTorques.js');
let ConstrainedOrientation = require('./ConstrainedOrientation.js');
let MapList = require('./MapList.js');
let SafetyNotificationList = require('./SafetyNotificationList.js');
let ConstrainedJointAngles = require('./ConstrainedJointAngles.js');
let SystemTime = require('./SystemTime.js');
let ProtectionZoneList = require('./ProtectionZoneList.js');
let Sequence = require('./Sequence.js');
let IPv4Configuration = require('./IPv4Configuration.js');
let MappingInfoNotification = require('./MappingInfoNotification.js');
let ControllerType = require('./ControllerType.js');
let TransformationMatrix = require('./TransformationMatrix.js');
let JointTrajectoryConstraint = require('./JointTrajectoryConstraint.js');
let Base_SafetyIdentifier = require('./Base_SafetyIdentifier.js');
let SequenceList = require('./SequenceList.js');
let ControllerConfigurationList = require('./ControllerConfigurationList.js');
let NetworkNotificationList = require('./NetworkNotificationList.js');
let PreComputedJointTrajectory = require('./PreComputedJointTrajectory.js');
let CartesianWaypoint = require('./CartesianWaypoint.js');
let ProtectionZoneNotificationList = require('./ProtectionZoneNotificationList.js');
let TrajectoryContinuityMode = require('./TrajectoryContinuityMode.js');
let BluetoothEnableState = require('./BluetoothEnableState.js');
let ChangeJointSpeeds = require('./ChangeJointSpeeds.js');
let Base_ControlModeNotification = require('./Base_ControlModeNotification.js');
let JointSpeed = require('./JointSpeed.js');
let Base_JointSpeeds = require('./Base_JointSpeeds.js');
let WristDigitalInputIdentifier = require('./WristDigitalInputIdentifier.js');
let Gripper = require('./Gripper.js');
let BridgeStatus = require('./BridgeStatus.js');
let AppendActionInformation = require('./AppendActionInformation.js');
let GpioPinConfiguration = require('./GpioPinConfiguration.js');
let Waypoint = require('./Waypoint.js');
let Faults = require('./Faults.js');
let ControllerElementHandle = require('./ControllerElementHandle.js');
let KinematicTrajectoryConstraints = require('./KinematicTrajectoryConstraints.js');
let UserNotification = require('./UserNotification.js');
let RFConfiguration = require('./RFConfiguration.js');
let NetworkType = require('./NetworkType.js');
let ActionType = require('./ActionType.js');
let SequenceInformation = require('./SequenceInformation.js');
let GpioBehavior = require('./GpioBehavior.js');
let Finger = require('./Finger.js');
let CartesianLimitation = require('./CartesianLimitation.js');
let ServoingMode = require('./ServoingMode.js');
let ConfigurationNotificationEvent = require('./ConfigurationNotificationEvent.js');
let GpioPinPropertyFlags = require('./GpioPinPropertyFlags.js');
let Waypoint_type_of_waypoint = require('./Waypoint_type_of_waypoint.js');
let MapEvent_events = require('./MapEvent_events.js');
let IKData = require('./IKData.js');
let ControllerElementState = require('./ControllerElementState.js');
let Snapshot = require('./Snapshot.js');
let SoundType = require('./SoundType.js');
let FullUserProfile = require('./FullUserProfile.js');
let SequenceTasksRange = require('./SequenceTasksRange.js');
let ControllerNotificationList = require('./ControllerNotificationList.js');
let SafetyEvent = require('./SafetyEvent.js');
let WaypointList = require('./WaypointList.js');
let JointTorque = require('./JointTorque.js');
let ActionNotification = require('./ActionNotification.js');
let BridgeIdentifier = require('./BridgeIdentifier.js');
let ControlModeNotificationList = require('./ControlModeNotificationList.js');
let OperatingMode = require('./OperatingMode.js');
let PreComputedJointTrajectoryElement = require('./PreComputedJointTrajectoryElement.js');
let ConstrainedPosition = require('./ConstrainedPosition.js');
let ServoingModeNotificationList = require('./ServoingModeNotificationList.js');
let IPv4Information = require('./IPv4Information.js');
let ControllerState = require('./ControllerState.js');
let CartesianSpeed = require('./CartesianSpeed.js');
let ConfigurationChangeNotification = require('./ConfigurationChangeNotification.js');
let GpioConfigurationList = require('./GpioConfigurationList.js');
let GpioCommand = require('./GpioCommand.js');
let Map = require('./Map.js');
let ActionEvent = require('./ActionEvent.js');
let ConstrainedJointAngle = require('./ConstrainedJointAngle.js');
let Pose = require('./Pose.js');
let ControllerNotification_state = require('./ControllerNotification_state.js');
let UserProfile = require('./UserProfile.js');
let BridgeConfig = require('./BridgeConfig.js');
let ConfigurationChangeNotificationList = require('./ConfigurationChangeNotificationList.js');
let TrajectoryErrorIdentifier = require('./TrajectoryErrorIdentifier.js');
let MapGroupList = require('./MapGroupList.js');
let Point = require('./Point.js');
let ControllerNotification = require('./ControllerNotification.js');
let UserList = require('./UserList.js');
let Admittance = require('./Admittance.js');
let JointAngle = require('./JointAngle.js');
let ControllerBehavior = require('./ControllerBehavior.js');
let Orientation = require('./Orientation.js');
let Base_Position = require('./Base_Position.js');
let Mapping = require('./Mapping.js');
let NetworkHandle = require('./NetworkHandle.js');
let WrenchCommand = require('./WrenchCommand.js');
let ControllerElementEventType = require('./ControllerElementEventType.js');
let AdmittanceMode = require('./AdmittanceMode.js');
let SwitchControlMapping = require('./SwitchControlMapping.js');
let Query = require('./Query.js');
let ConstrainedPose = require('./ConstrainedPose.js');
let EmergencyStop = require('./EmergencyStop.js');
let Timeout = require('./Timeout.js');
let WifiEncryptionType = require('./WifiEncryptionType.js');
let EventIdSequenceInfoNotification = require('./EventIdSequenceInfoNotification.js');
let Action = require('./Action.js');
let CartesianTrajectoryConstraint = require('./CartesianTrajectoryConstraint.js');
let Base_ControlModeInformation = require('./Base_ControlModeInformation.js');
let JointAngles = require('./JointAngles.js');
let ActuatorInformation = require('./ActuatorInformation.js');
let NetworkEvent = require('./NetworkEvent.js');
let ServoingModeNotification = require('./ServoingModeNotification.js');
let SnapshotType = require('./SnapshotType.js');
let ActionList = require('./ActionList.js');
let ArmStateInformation = require('./ArmStateInformation.js');
let ChangeWrench = require('./ChangeWrench.js');
let TrajectoryInfo = require('./TrajectoryInfo.js');
let GpioAction = require('./GpioAction.js');
let ShapeType = require('./ShapeType.js');
let WrenchLimitation = require('./WrenchLimitation.js');
let BridgePortConfig = require('./BridgePortConfig.js');
let TransformationRow = require('./TransformationRow.js');
let ConfigurationChangeNotification_configuration_change = require('./ConfigurationChangeNotification_configuration_change.js');
let ActivateMapHandle = require('./ActivateMapHandle.js');
let RobotEventNotification = require('./RobotEventNotification.js');
let GripperMode = require('./GripperMode.js');
let UserEvent = require('./UserEvent.js');
let FactoryEvent = require('./FactoryEvent.js');
let TwistCommand = require('./TwistCommand.js');
let BridgeType = require('./BridgeType.js');
let ProtectionZoneNotification = require('./ProtectionZoneNotification.js');
let ZoneShape = require('./ZoneShape.js');
let MapEvent = require('./MapEvent.js');
let ProtectionZoneEvent = require('./ProtectionZoneEvent.js');
let SequenceTaskHandle = require('./SequenceTaskHandle.js');
let FactoryNotification = require('./FactoryNotification.js');
let ActionExecutionState = require('./ActionExecutionState.js');
let TwistLimitation = require('./TwistLimitation.js');
let ControllerInputType = require('./ControllerInputType.js');
let ControllerEvent = require('./ControllerEvent.js');
let ProtectionZoneHandle = require('./ProtectionZoneHandle.js');
let SignalQuality = require('./SignalQuality.js');
let MappingInfoNotificationList = require('./MappingInfoNotificationList.js');
let FirmwareBundleVersions = require('./FirmwareBundleVersions.js');
let Wrench = require('./Wrench.js');
let MapGroupHandle = require('./MapGroupHandle.js');
let MappingList = require('./MappingList.js');
let RobotEventNotificationList = require('./RobotEventNotificationList.js');
let UserProfileList = require('./UserProfileList.js');
let ControllerHandle = require('./ControllerHandle.js');
let Xbox360DigitalInputIdentifier = require('./Xbox360DigitalInputIdentifier.js');
let BackupEvent = require('./BackupEvent.js');
let Base_GpioConfiguration = require('./Base_GpioConfiguration.js');
let TrajectoryErrorType = require('./TrajectoryErrorType.js');
let RequestedActionType = require('./RequestedActionType.js');
let JointsLimitationsList = require('./JointsLimitationsList.js');
let WifiInformationList = require('./WifiInformationList.js');
let MapGroup = require('./MapGroup.js');
let MapHandle = require('./MapHandle.js');
let WifiSecurityType = require('./WifiSecurityType.js');
let PasswordChange = require('./PasswordChange.js');
let Ssid = require('./Ssid.js');
let AngularWaypoint = require('./AngularWaypoint.js');
let BridgeResult = require('./BridgeResult.js');
let SequenceInfoNotification = require('./SequenceInfoNotification.js');
let Delay = require('./Delay.js');
let Base_Stop = require('./Base_Stop.js');
let TrajectoryInfoType = require('./TrajectoryInfoType.js');
let OperatingModeNotificationList = require('./OperatingModeNotificationList.js');
let LedState = require('./LedState.js');
let JointLimitation = require('./JointLimitation.js');
let CommunicationInterfaceConfiguration = require('./CommunicationInterfaceConfiguration.js');
let FullIPv4Configuration = require('./FullIPv4Configuration.js');
let CartesianTrajectoryConstraint_type = require('./CartesianTrajectoryConstraint_type.js');
let ProtectionZoneInformation = require('./ProtectionZoneInformation.js');
let MapElement = require('./MapElement.js');
let TrajectoryErrorElement = require('./TrajectoryErrorElement.js');
let SequenceInfoNotificationList = require('./SequenceInfoNotificationList.js');
let UserNotificationList = require('./UserNotificationList.js');
let SequenceTasks = require('./SequenceTasks.js');
let AdvancedSequenceHandle = require('./AdvancedSequenceHandle.js');
let OperatingModeInformation = require('./OperatingModeInformation.js');
let SequenceTask = require('./SequenceTask.js');
let LimitationType = require('./LimitationType.js');
let FirmwareComponentVersion = require('./FirmwareComponentVersion.js');
let SequenceHandle = require('./SequenceHandle.js');
let SequenceTaskConfiguration = require('./SequenceTaskConfiguration.js');
let Base_RotationMatrix = require('./Base_RotationMatrix.js');
let ControllerElementHandle_identifier = require('./ControllerElementHandle_identifier.js');
let CartesianLimitationList = require('./CartesianLimitationList.js');
let WifiInformation = require('./WifiInformation.js');
let ArmStateNotification = require('./ArmStateNotification.js');
let GripperRequest = require('./GripperRequest.js');
let RobotEvent = require('./RobotEvent.js');
let ServoingModeInformation = require('./ServoingModeInformation.js');
let GripperCommand = require('./GripperCommand.js');
let ProtectionZone = require('./ProtectionZone.js');
let BridgeList = require('./BridgeList.js');
let ControllerList = require('./ControllerList.js');
let WifiConfiguration = require('./WifiConfiguration.js');
let Xbox360AnalogInputIdentifier = require('./Xbox360AnalogInputIdentifier.js');
let WifiEnableState = require('./WifiEnableState.js');
let Gen3GpioPinId = require('./Gen3GpioPinId.js');
let JointNavigationDirection = require('./JointNavigationDirection.js');
let WaypointValidationReport = require('./WaypointValidationReport.js');
let Action_action_parameters = require('./Action_action_parameters.js');
let MappingHandle = require('./MappingHandle.js');
let Base_CapSenseConfig = require('./Base_CapSenseConfig.js');
let WifiConfigurationList = require('./WifiConfigurationList.js');
let Base_RotationMatrixRow = require('./Base_RotationMatrixRow.js');
let NavigationDirection = require('./NavigationDirection.js');
let TrajectoryErrorReport = require('./TrajectoryErrorReport.js');
let SequenceTasksPair = require('./SequenceTasksPair.js');
let Base_CapSenseMode = require('./Base_CapSenseMode.js');
let ActionHandle = require('./ActionHandle.js');
let Base_ControlMode = require('./Base_ControlMode.js');
let ControllerEventType = require('./ControllerEventType.js');
let ControllerConfiguration = require('./ControllerConfiguration.js');
let WrenchMode = require('./WrenchMode.js');
let NetworkNotification = require('./NetworkNotification.js');
let Twist = require('./Twist.js');
let ChangeTwist = require('./ChangeTwist.js');
let SequenceTasksConfiguration = require('./SequenceTasksConfiguration.js');
let Base_ServiceVersion = require('./Base_ServiceVersion.js');
let BaseCyclic_CustomData = require('./BaseCyclic_CustomData.js');
let ActuatorCustomData = require('./ActuatorCustomData.js');
let ActuatorFeedback = require('./ActuatorFeedback.js');
let BaseCyclic_Feedback = require('./BaseCyclic_Feedback.js');
let BaseFeedback = require('./BaseFeedback.js');
let BaseCyclic_ServiceVersion = require('./BaseCyclic_ServiceVersion.js');
let ActuatorCommand = require('./ActuatorCommand.js');
let BaseCyclic_Command = require('./BaseCyclic_Command.js');
let NotificationOptions = require('./NotificationOptions.js');
let CountryCode = require('./CountryCode.js');
let UARTSpeed = require('./UARTSpeed.js');
let CountryCodeIdentifier = require('./CountryCodeIdentifier.js');
let UARTDeviceIdentification = require('./UARTDeviceIdentification.js');
let NotificationHandle = require('./NotificationHandle.js');
let UARTParity = require('./UARTParity.js');
let SafetyHandle = require('./SafetyHandle.js');
let UserProfileHandle = require('./UserProfileHandle.js');
let SafetyNotification = require('./SafetyNotification.js');
let Timestamp = require('./Timestamp.js');
let CartesianReferenceFrame = require('./CartesianReferenceFrame.js');
let UARTStopBits = require('./UARTStopBits.js');
let Permission = require('./Permission.js');
let UARTWordLength = require('./UARTWordLength.js');
let Empty = require('./Empty.js');
let UARTConfiguration = require('./UARTConfiguration.js');
let Connection = require('./Connection.js');
let Unit = require('./Unit.js');
let SafetyStatusValue = require('./SafetyStatusValue.js');
let DeviceHandle = require('./DeviceHandle.js');
let NotificationType = require('./NotificationType.js');
let ArmState = require('./ArmState.js');
let DeviceTypes = require('./DeviceTypes.js');
let TwistAngularSoftLimit = require('./TwistAngularSoftLimit.js');
let KinematicLimitsList = require('./KinematicLimitsList.js');
let GravityVector = require('./GravityVector.js');
let LinearTwist = require('./LinearTwist.js');
let ControlConfig_Position = require('./ControlConfig_Position.js');
let DesiredSpeeds = require('./DesiredSpeeds.js');
let AngularTwist = require('./AngularTwist.js');
let JointAccelerationSoftLimits = require('./JointAccelerationSoftLimits.js');
let TwistLinearSoftLimit = require('./TwistLinearSoftLimit.js');
let ControlConfigurationNotification = require('./ControlConfigurationNotification.js');
let CartesianTransform = require('./CartesianTransform.js');
let ControlConfig_JointSpeeds = require('./ControlConfig_JointSpeeds.js');
let ControlConfig_ControlModeNotification = require('./ControlConfig_ControlModeNotification.js');
let ControlConfig_ControlModeInformation = require('./ControlConfig_ControlModeInformation.js');
let JointSpeedSoftLimits = require('./JointSpeedSoftLimits.js');
let PayloadInformation = require('./PayloadInformation.js');
let KinematicLimits = require('./KinematicLimits.js');
let ControlConfig_ServiceVersion = require('./ControlConfig_ServiceVersion.js');
let ToolConfiguration = require('./ToolConfiguration.js');
let ControlConfigurationEvent = require('./ControlConfigurationEvent.js');
let ControlConfig_ControlMode = require('./ControlConfig_ControlMode.js');
let CartesianReferenceFrameInfo = require('./CartesianReferenceFrameInfo.js');
let CalibrationResult = require('./CalibrationResult.js');
let CalibrationItem = require('./CalibrationItem.js');
let PartNumber = require('./PartNumber.js');
let SafetyStatus = require('./SafetyStatus.js');
let RebootRqst = require('./RebootRqst.js');
let DeviceConfig_CapSenseMode = require('./DeviceConfig_CapSenseMode.js');
let DeviceType = require('./DeviceType.js');
let SafetyThreshold = require('./SafetyThreshold.js');
let SerialNumber = require('./SerialNumber.js');
let CalibrationParameter = require('./CalibrationParameter.js');
let ModelNumber = require('./ModelNumber.js');
let CalibrationStatus = require('./CalibrationStatus.js');
let SafetyInformation = require('./SafetyInformation.js');
let RunModes = require('./RunModes.js');
let SafetyEnable = require('./SafetyEnable.js');
let CalibrationParameter_value = require('./CalibrationParameter_value.js');
let CapSenseRegister = require('./CapSenseRegister.js');
let MACAddress = require('./MACAddress.js');
let CalibrationElement = require('./CalibrationElement.js');
let BootloaderVersion = require('./BootloaderVersion.js');
let PartNumberRevision = require('./PartNumberRevision.js');
let DeviceConfig_CapSenseConfig = require('./DeviceConfig_CapSenseConfig.js');
let PowerOnSelfTestResult = require('./PowerOnSelfTestResult.js');
let DeviceConfig_ServiceVersion = require('./DeviceConfig_ServiceVersion.js');
let IPv4Settings = require('./IPv4Settings.js');
let RunMode = require('./RunMode.js');
let FirmwareVersion = require('./FirmwareVersion.js');
let DeviceConfig_SafetyLimitType = require('./DeviceConfig_SafetyLimitType.js');
let SafetyConfiguration = require('./SafetyConfiguration.js');
let SafetyConfigurationList = require('./SafetyConfigurationList.js');
let Calibration = require('./Calibration.js');
let SafetyInformationList = require('./SafetyInformationList.js');
let DeviceManager_ServiceVersion = require('./DeviceManager_ServiceVersion.js');
let DeviceHandles = require('./DeviceHandles.js');
let GripperConfig_SafetyIdentifier = require('./GripperConfig_SafetyIdentifier.js');
let RobotiqGripperStatusFlags = require('./RobotiqGripperStatusFlags.js');
let CustomDataUnit = require('./CustomDataUnit.js');
let MotorFeedback = require('./MotorFeedback.js');
let GripperCyclic_ServiceVersion = require('./GripperCyclic_ServiceVersion.js');
let GripperCyclic_Command = require('./GripperCyclic_Command.js');
let GripperCyclic_CustomData = require('./GripperCyclic_CustomData.js');
let GripperCyclic_MessageId = require('./GripperCyclic_MessageId.js');
let GripperCyclic_Feedback = require('./GripperCyclic_Feedback.js');
let MotorCommand = require('./MotorCommand.js');
let GPIOValue = require('./GPIOValue.js');
let InterconnectConfig_SafetyIdentifier = require('./InterconnectConfig_SafetyIdentifier.js');
let GPIOIdentifier = require('./GPIOIdentifier.js');
let EthernetDuplex = require('./EthernetDuplex.js');
let GPIOPull = require('./GPIOPull.js');
let I2CConfiguration = require('./I2CConfiguration.js');
let I2CDevice = require('./I2CDevice.js');
let UARTPortId = require('./UARTPortId.js');
let EthernetSpeed = require('./EthernetSpeed.js');
let I2CRegisterAddressSize = require('./I2CRegisterAddressSize.js');
let EthernetDevice = require('./EthernetDevice.js');
let I2CWriteParameter = require('./I2CWriteParameter.js');
let I2CData = require('./I2CData.js');
let GPIOIdentification = require('./GPIOIdentification.js');
let EthernetDeviceIdentification = require('./EthernetDeviceIdentification.js');
let EthernetConfiguration = require('./EthernetConfiguration.js');
let InterconnectConfig_GPIOConfiguration = require('./InterconnectConfig_GPIOConfiguration.js');
let I2CWriteRegisterParameter = require('./I2CWriteRegisterParameter.js');
let I2CReadParameter = require('./I2CReadParameter.js');
let I2CReadRegisterParameter = require('./I2CReadRegisterParameter.js');
let I2CMode = require('./I2CMode.js');
let InterconnectConfig_ServiceVersion = require('./InterconnectConfig_ServiceVersion.js');
let GPIOState = require('./GPIOState.js');
let GPIOMode = require('./GPIOMode.js');
let I2CDeviceIdentification = require('./I2CDeviceIdentification.js');
let I2CDeviceAddressing = require('./I2CDeviceAddressing.js');
let InterconnectCyclic_Command = require('./InterconnectCyclic_Command.js');
let InterconnectCyclic_MessageId = require('./InterconnectCyclic_MessageId.js');
let InterconnectCyclic_Feedback_tool_feedback = require('./InterconnectCyclic_Feedback_tool_feedback.js');
let InterconnectCyclic_Feedback = require('./InterconnectCyclic_Feedback.js');
let InterconnectCyclic_Command_tool_command = require('./InterconnectCyclic_Command_tool_command.js');
let InterconnectCyclic_ServiceVersion = require('./InterconnectCyclic_ServiceVersion.js');
let InterconnectCyclic_CustomData = require('./InterconnectCyclic_CustomData.js');
let InterconnectCyclic_CustomData_tool_customData = require('./InterconnectCyclic_CustomData_tool_customData.js');
let WristType = require('./WristType.js');
let ModelId = require('./ModelId.js');
let ArmLaterality = require('./ArmLaterality.js');
let ProductConfigurationEndEffectorType = require('./ProductConfigurationEndEffectorType.js');
let BrakeType = require('./BrakeType.js');
let BaseType = require('./BaseType.js');
let CompleteProductConfiguration = require('./CompleteProductConfiguration.js');
let VisionModuleType = require('./VisionModuleType.js');
let InterfaceModuleType = require('./InterfaceModuleType.js');
let EndEffectorType = require('./EndEffectorType.js');
let BitRate = require('./BitRate.js');
let SensorIdentifier = require('./SensorIdentifier.js');
let OptionValue = require('./OptionValue.js');
let ExtrinsicParameters = require('./ExtrinsicParameters.js');
let FocusAction = require('./FocusAction.js');
let SensorFocusAction = require('./SensorFocusAction.js');
let VisionConfig_RotationMatrixRow = require('./VisionConfig_RotationMatrixRow.js');
let Resolution = require('./Resolution.js');
let IntrinsicParameters = require('./IntrinsicParameters.js');
let VisionNotification = require('./VisionNotification.js');
let ManualFocus = require('./ManualFocus.js');
let FrameRate = require('./FrameRate.js');
let OptionInformation = require('./OptionInformation.js');
let OptionIdentifier = require('./OptionIdentifier.js');
let TranslationVector = require('./TranslationVector.js');
let SensorSettings = require('./SensorSettings.js');
let SensorFocusAction_action_parameters = require('./SensorFocusAction_action_parameters.js');
let VisionConfig_RotationMatrix = require('./VisionConfig_RotationMatrix.js');
let DistortionCoefficients = require('./DistortionCoefficients.js');
let FocusPoint = require('./FocusPoint.js');
let Sensor = require('./Sensor.js');
let VisionEvent = require('./VisionEvent.js');
let VisionConfig_ServiceVersion = require('./VisionConfig_ServiceVersion.js');
let IntrinsicProfileIdentifier = require('./IntrinsicProfileIdentifier.js');
let Option = require('./Option.js');
let FollowCartesianTrajectoryResult = require('./FollowCartesianTrajectoryResult.js');
let FollowCartesianTrajectoryFeedback = require('./FollowCartesianTrajectoryFeedback.js');
let FollowCartesianTrajectoryActionFeedback = require('./FollowCartesianTrajectoryActionFeedback.js');
let FollowCartesianTrajectoryAction = require('./FollowCartesianTrajectoryAction.js');
let FollowCartesianTrajectoryActionGoal = require('./FollowCartesianTrajectoryActionGoal.js');
let FollowCartesianTrajectoryActionResult = require('./FollowCartesianTrajectoryActionResult.js');
let FollowCartesianTrajectoryGoal = require('./FollowCartesianTrajectoryGoal.js');

module.exports = {
  ApiOptions: ApiOptions,
  KortexError: KortexError,
  ErrorCodes: ErrorCodes,
  SubErrorCodes: SubErrorCodes,
  CoggingFeedforwardModeInformation: CoggingFeedforwardModeInformation,
  ActuatorConfig_SafetyLimitType: ActuatorConfig_SafetyLimitType,
  VectorDriveParameters: VectorDriveParameters,
  ActuatorConfig_ServiceVersion: ActuatorConfig_ServiceVersion,
  TorqueOffset: TorqueOffset,
  Servoing: Servoing,
  CommandMode: CommandMode,
  CustomDataSelection: CustomDataSelection,
  CommandModeInformation: CommandModeInformation,
  StepResponse: StepResponse,
  ActuatorConfig_ControlModeInformation: ActuatorConfig_ControlModeInformation,
  RampResponse: RampResponse,
  AxisOffsets: AxisOffsets,
  TorqueCalibration: TorqueCalibration,
  EncoderDerivativeParameters: EncoderDerivativeParameters,
  CustomDataIndex: CustomDataIndex,
  PositionCommand: PositionCommand,
  LoopSelection: LoopSelection,
  SafetyIdentifierBankA: SafetyIdentifierBankA,
  ControlLoopParameters: ControlLoopParameters,
  ControlLoopSelection: ControlLoopSelection,
  CoggingFeedforwardMode: CoggingFeedforwardMode,
  ControlLoop: ControlLoop,
  FrequencyResponse: FrequencyResponse,
  AxisPosition: AxisPosition,
  ActuatorConfig_ControlMode: ActuatorConfig_ControlMode,
  ActuatorCyclic_MessageId: ActuatorCyclic_MessageId,
  ActuatorCyclic_CustomData: ActuatorCyclic_CustomData,
  CommandFlags: CommandFlags,
  ActuatorCyclic_Feedback: ActuatorCyclic_Feedback,
  StatusFlags: StatusFlags,
  ActuatorCyclic_ServiceVersion: ActuatorCyclic_ServiceVersion,
  ActuatorCyclic_Command: ActuatorCyclic_Command,
  ActionNotificationList: ActionNotificationList,
  GpioEvent: GpioEvent,
  OperatingModeNotification: OperatingModeNotification,
  ControllerConfigurationMode: ControllerConfigurationMode,
  JointTrajectoryConstraintType: JointTrajectoryConstraintType,
  JointTorques: JointTorques,
  ConstrainedOrientation: ConstrainedOrientation,
  MapList: MapList,
  SafetyNotificationList: SafetyNotificationList,
  ConstrainedJointAngles: ConstrainedJointAngles,
  SystemTime: SystemTime,
  ProtectionZoneList: ProtectionZoneList,
  Sequence: Sequence,
  IPv4Configuration: IPv4Configuration,
  MappingInfoNotification: MappingInfoNotification,
  ControllerType: ControllerType,
  TransformationMatrix: TransformationMatrix,
  JointTrajectoryConstraint: JointTrajectoryConstraint,
  Base_SafetyIdentifier: Base_SafetyIdentifier,
  SequenceList: SequenceList,
  ControllerConfigurationList: ControllerConfigurationList,
  NetworkNotificationList: NetworkNotificationList,
  PreComputedJointTrajectory: PreComputedJointTrajectory,
  CartesianWaypoint: CartesianWaypoint,
  ProtectionZoneNotificationList: ProtectionZoneNotificationList,
  TrajectoryContinuityMode: TrajectoryContinuityMode,
  BluetoothEnableState: BluetoothEnableState,
  ChangeJointSpeeds: ChangeJointSpeeds,
  Base_ControlModeNotification: Base_ControlModeNotification,
  JointSpeed: JointSpeed,
  Base_JointSpeeds: Base_JointSpeeds,
  WristDigitalInputIdentifier: WristDigitalInputIdentifier,
  Gripper: Gripper,
  BridgeStatus: BridgeStatus,
  AppendActionInformation: AppendActionInformation,
  GpioPinConfiguration: GpioPinConfiguration,
  Waypoint: Waypoint,
  Faults: Faults,
  ControllerElementHandle: ControllerElementHandle,
  KinematicTrajectoryConstraints: KinematicTrajectoryConstraints,
  UserNotification: UserNotification,
  RFConfiguration: RFConfiguration,
  NetworkType: NetworkType,
  ActionType: ActionType,
  SequenceInformation: SequenceInformation,
  GpioBehavior: GpioBehavior,
  Finger: Finger,
  CartesianLimitation: CartesianLimitation,
  ServoingMode: ServoingMode,
  ConfigurationNotificationEvent: ConfigurationNotificationEvent,
  GpioPinPropertyFlags: GpioPinPropertyFlags,
  Waypoint_type_of_waypoint: Waypoint_type_of_waypoint,
  MapEvent_events: MapEvent_events,
  IKData: IKData,
  ControllerElementState: ControllerElementState,
  Snapshot: Snapshot,
  SoundType: SoundType,
  FullUserProfile: FullUserProfile,
  SequenceTasksRange: SequenceTasksRange,
  ControllerNotificationList: ControllerNotificationList,
  SafetyEvent: SafetyEvent,
  WaypointList: WaypointList,
  JointTorque: JointTorque,
  ActionNotification: ActionNotification,
  BridgeIdentifier: BridgeIdentifier,
  ControlModeNotificationList: ControlModeNotificationList,
  OperatingMode: OperatingMode,
  PreComputedJointTrajectoryElement: PreComputedJointTrajectoryElement,
  ConstrainedPosition: ConstrainedPosition,
  ServoingModeNotificationList: ServoingModeNotificationList,
  IPv4Information: IPv4Information,
  ControllerState: ControllerState,
  CartesianSpeed: CartesianSpeed,
  ConfigurationChangeNotification: ConfigurationChangeNotification,
  GpioConfigurationList: GpioConfigurationList,
  GpioCommand: GpioCommand,
  Map: Map,
  ActionEvent: ActionEvent,
  ConstrainedJointAngle: ConstrainedJointAngle,
  Pose: Pose,
  ControllerNotification_state: ControllerNotification_state,
  UserProfile: UserProfile,
  BridgeConfig: BridgeConfig,
  ConfigurationChangeNotificationList: ConfigurationChangeNotificationList,
  TrajectoryErrorIdentifier: TrajectoryErrorIdentifier,
  MapGroupList: MapGroupList,
  Point: Point,
  ControllerNotification: ControllerNotification,
  UserList: UserList,
  Admittance: Admittance,
  JointAngle: JointAngle,
  ControllerBehavior: ControllerBehavior,
  Orientation: Orientation,
  Base_Position: Base_Position,
  Mapping: Mapping,
  NetworkHandle: NetworkHandle,
  WrenchCommand: WrenchCommand,
  ControllerElementEventType: ControllerElementEventType,
  AdmittanceMode: AdmittanceMode,
  SwitchControlMapping: SwitchControlMapping,
  Query: Query,
  ConstrainedPose: ConstrainedPose,
  EmergencyStop: EmergencyStop,
  Timeout: Timeout,
  WifiEncryptionType: WifiEncryptionType,
  EventIdSequenceInfoNotification: EventIdSequenceInfoNotification,
  Action: Action,
  CartesianTrajectoryConstraint: CartesianTrajectoryConstraint,
  Base_ControlModeInformation: Base_ControlModeInformation,
  JointAngles: JointAngles,
  ActuatorInformation: ActuatorInformation,
  NetworkEvent: NetworkEvent,
  ServoingModeNotification: ServoingModeNotification,
  SnapshotType: SnapshotType,
  ActionList: ActionList,
  ArmStateInformation: ArmStateInformation,
  ChangeWrench: ChangeWrench,
  TrajectoryInfo: TrajectoryInfo,
  GpioAction: GpioAction,
  ShapeType: ShapeType,
  WrenchLimitation: WrenchLimitation,
  BridgePortConfig: BridgePortConfig,
  TransformationRow: TransformationRow,
  ConfigurationChangeNotification_configuration_change: ConfigurationChangeNotification_configuration_change,
  ActivateMapHandle: ActivateMapHandle,
  RobotEventNotification: RobotEventNotification,
  GripperMode: GripperMode,
  UserEvent: UserEvent,
  FactoryEvent: FactoryEvent,
  TwistCommand: TwistCommand,
  BridgeType: BridgeType,
  ProtectionZoneNotification: ProtectionZoneNotification,
  ZoneShape: ZoneShape,
  MapEvent: MapEvent,
  ProtectionZoneEvent: ProtectionZoneEvent,
  SequenceTaskHandle: SequenceTaskHandle,
  FactoryNotification: FactoryNotification,
  ActionExecutionState: ActionExecutionState,
  TwistLimitation: TwistLimitation,
  ControllerInputType: ControllerInputType,
  ControllerEvent: ControllerEvent,
  ProtectionZoneHandle: ProtectionZoneHandle,
  SignalQuality: SignalQuality,
  MappingInfoNotificationList: MappingInfoNotificationList,
  FirmwareBundleVersions: FirmwareBundleVersions,
  Wrench: Wrench,
  MapGroupHandle: MapGroupHandle,
  MappingList: MappingList,
  RobotEventNotificationList: RobotEventNotificationList,
  UserProfileList: UserProfileList,
  ControllerHandle: ControllerHandle,
  Xbox360DigitalInputIdentifier: Xbox360DigitalInputIdentifier,
  BackupEvent: BackupEvent,
  Base_GpioConfiguration: Base_GpioConfiguration,
  TrajectoryErrorType: TrajectoryErrorType,
  RequestedActionType: RequestedActionType,
  JointsLimitationsList: JointsLimitationsList,
  WifiInformationList: WifiInformationList,
  MapGroup: MapGroup,
  MapHandle: MapHandle,
  WifiSecurityType: WifiSecurityType,
  PasswordChange: PasswordChange,
  Ssid: Ssid,
  AngularWaypoint: AngularWaypoint,
  BridgeResult: BridgeResult,
  SequenceInfoNotification: SequenceInfoNotification,
  Delay: Delay,
  Base_Stop: Base_Stop,
  TrajectoryInfoType: TrajectoryInfoType,
  OperatingModeNotificationList: OperatingModeNotificationList,
  LedState: LedState,
  JointLimitation: JointLimitation,
  CommunicationInterfaceConfiguration: CommunicationInterfaceConfiguration,
  FullIPv4Configuration: FullIPv4Configuration,
  CartesianTrajectoryConstraint_type: CartesianTrajectoryConstraint_type,
  ProtectionZoneInformation: ProtectionZoneInformation,
  MapElement: MapElement,
  TrajectoryErrorElement: TrajectoryErrorElement,
  SequenceInfoNotificationList: SequenceInfoNotificationList,
  UserNotificationList: UserNotificationList,
  SequenceTasks: SequenceTasks,
  AdvancedSequenceHandle: AdvancedSequenceHandle,
  OperatingModeInformation: OperatingModeInformation,
  SequenceTask: SequenceTask,
  LimitationType: LimitationType,
  FirmwareComponentVersion: FirmwareComponentVersion,
  SequenceHandle: SequenceHandle,
  SequenceTaskConfiguration: SequenceTaskConfiguration,
  Base_RotationMatrix: Base_RotationMatrix,
  ControllerElementHandle_identifier: ControllerElementHandle_identifier,
  CartesianLimitationList: CartesianLimitationList,
  WifiInformation: WifiInformation,
  ArmStateNotification: ArmStateNotification,
  GripperRequest: GripperRequest,
  RobotEvent: RobotEvent,
  ServoingModeInformation: ServoingModeInformation,
  GripperCommand: GripperCommand,
  ProtectionZone: ProtectionZone,
  BridgeList: BridgeList,
  ControllerList: ControllerList,
  WifiConfiguration: WifiConfiguration,
  Xbox360AnalogInputIdentifier: Xbox360AnalogInputIdentifier,
  WifiEnableState: WifiEnableState,
  Gen3GpioPinId: Gen3GpioPinId,
  JointNavigationDirection: JointNavigationDirection,
  WaypointValidationReport: WaypointValidationReport,
  Action_action_parameters: Action_action_parameters,
  MappingHandle: MappingHandle,
  Base_CapSenseConfig: Base_CapSenseConfig,
  WifiConfigurationList: WifiConfigurationList,
  Base_RotationMatrixRow: Base_RotationMatrixRow,
  NavigationDirection: NavigationDirection,
  TrajectoryErrorReport: TrajectoryErrorReport,
  SequenceTasksPair: SequenceTasksPair,
  Base_CapSenseMode: Base_CapSenseMode,
  ActionHandle: ActionHandle,
  Base_ControlMode: Base_ControlMode,
  ControllerEventType: ControllerEventType,
  ControllerConfiguration: ControllerConfiguration,
  WrenchMode: WrenchMode,
  NetworkNotification: NetworkNotification,
  Twist: Twist,
  ChangeTwist: ChangeTwist,
  SequenceTasksConfiguration: SequenceTasksConfiguration,
  Base_ServiceVersion: Base_ServiceVersion,
  BaseCyclic_CustomData: BaseCyclic_CustomData,
  ActuatorCustomData: ActuatorCustomData,
  ActuatorFeedback: ActuatorFeedback,
  BaseCyclic_Feedback: BaseCyclic_Feedback,
  BaseFeedback: BaseFeedback,
  BaseCyclic_ServiceVersion: BaseCyclic_ServiceVersion,
  ActuatorCommand: ActuatorCommand,
  BaseCyclic_Command: BaseCyclic_Command,
  NotificationOptions: NotificationOptions,
  CountryCode: CountryCode,
  UARTSpeed: UARTSpeed,
  CountryCodeIdentifier: CountryCodeIdentifier,
  UARTDeviceIdentification: UARTDeviceIdentification,
  NotificationHandle: NotificationHandle,
  UARTParity: UARTParity,
  SafetyHandle: SafetyHandle,
  UserProfileHandle: UserProfileHandle,
  SafetyNotification: SafetyNotification,
  Timestamp: Timestamp,
  CartesianReferenceFrame: CartesianReferenceFrame,
  UARTStopBits: UARTStopBits,
  Permission: Permission,
  UARTWordLength: UARTWordLength,
  Empty: Empty,
  UARTConfiguration: UARTConfiguration,
  Connection: Connection,
  Unit: Unit,
  SafetyStatusValue: SafetyStatusValue,
  DeviceHandle: DeviceHandle,
  NotificationType: NotificationType,
  ArmState: ArmState,
  DeviceTypes: DeviceTypes,
  TwistAngularSoftLimit: TwistAngularSoftLimit,
  KinematicLimitsList: KinematicLimitsList,
  GravityVector: GravityVector,
  LinearTwist: LinearTwist,
  ControlConfig_Position: ControlConfig_Position,
  DesiredSpeeds: DesiredSpeeds,
  AngularTwist: AngularTwist,
  JointAccelerationSoftLimits: JointAccelerationSoftLimits,
  TwistLinearSoftLimit: TwistLinearSoftLimit,
  ControlConfigurationNotification: ControlConfigurationNotification,
  CartesianTransform: CartesianTransform,
  ControlConfig_JointSpeeds: ControlConfig_JointSpeeds,
  ControlConfig_ControlModeNotification: ControlConfig_ControlModeNotification,
  ControlConfig_ControlModeInformation: ControlConfig_ControlModeInformation,
  JointSpeedSoftLimits: JointSpeedSoftLimits,
  PayloadInformation: PayloadInformation,
  KinematicLimits: KinematicLimits,
  ControlConfig_ServiceVersion: ControlConfig_ServiceVersion,
  ToolConfiguration: ToolConfiguration,
  ControlConfigurationEvent: ControlConfigurationEvent,
  ControlConfig_ControlMode: ControlConfig_ControlMode,
  CartesianReferenceFrameInfo: CartesianReferenceFrameInfo,
  CalibrationResult: CalibrationResult,
  CalibrationItem: CalibrationItem,
  PartNumber: PartNumber,
  SafetyStatus: SafetyStatus,
  RebootRqst: RebootRqst,
  DeviceConfig_CapSenseMode: DeviceConfig_CapSenseMode,
  DeviceType: DeviceType,
  SafetyThreshold: SafetyThreshold,
  SerialNumber: SerialNumber,
  CalibrationParameter: CalibrationParameter,
  ModelNumber: ModelNumber,
  CalibrationStatus: CalibrationStatus,
  SafetyInformation: SafetyInformation,
  RunModes: RunModes,
  SafetyEnable: SafetyEnable,
  CalibrationParameter_value: CalibrationParameter_value,
  CapSenseRegister: CapSenseRegister,
  MACAddress: MACAddress,
  CalibrationElement: CalibrationElement,
  BootloaderVersion: BootloaderVersion,
  PartNumberRevision: PartNumberRevision,
  DeviceConfig_CapSenseConfig: DeviceConfig_CapSenseConfig,
  PowerOnSelfTestResult: PowerOnSelfTestResult,
  DeviceConfig_ServiceVersion: DeviceConfig_ServiceVersion,
  IPv4Settings: IPv4Settings,
  RunMode: RunMode,
  FirmwareVersion: FirmwareVersion,
  DeviceConfig_SafetyLimitType: DeviceConfig_SafetyLimitType,
  SafetyConfiguration: SafetyConfiguration,
  SafetyConfigurationList: SafetyConfigurationList,
  Calibration: Calibration,
  SafetyInformationList: SafetyInformationList,
  DeviceManager_ServiceVersion: DeviceManager_ServiceVersion,
  DeviceHandles: DeviceHandles,
  GripperConfig_SafetyIdentifier: GripperConfig_SafetyIdentifier,
  RobotiqGripperStatusFlags: RobotiqGripperStatusFlags,
  CustomDataUnit: CustomDataUnit,
  MotorFeedback: MotorFeedback,
  GripperCyclic_ServiceVersion: GripperCyclic_ServiceVersion,
  GripperCyclic_Command: GripperCyclic_Command,
  GripperCyclic_CustomData: GripperCyclic_CustomData,
  GripperCyclic_MessageId: GripperCyclic_MessageId,
  GripperCyclic_Feedback: GripperCyclic_Feedback,
  MotorCommand: MotorCommand,
  GPIOValue: GPIOValue,
  InterconnectConfig_SafetyIdentifier: InterconnectConfig_SafetyIdentifier,
  GPIOIdentifier: GPIOIdentifier,
  EthernetDuplex: EthernetDuplex,
  GPIOPull: GPIOPull,
  I2CConfiguration: I2CConfiguration,
  I2CDevice: I2CDevice,
  UARTPortId: UARTPortId,
  EthernetSpeed: EthernetSpeed,
  I2CRegisterAddressSize: I2CRegisterAddressSize,
  EthernetDevice: EthernetDevice,
  I2CWriteParameter: I2CWriteParameter,
  I2CData: I2CData,
  GPIOIdentification: GPIOIdentification,
  EthernetDeviceIdentification: EthernetDeviceIdentification,
  EthernetConfiguration: EthernetConfiguration,
  InterconnectConfig_GPIOConfiguration: InterconnectConfig_GPIOConfiguration,
  I2CWriteRegisterParameter: I2CWriteRegisterParameter,
  I2CReadParameter: I2CReadParameter,
  I2CReadRegisterParameter: I2CReadRegisterParameter,
  I2CMode: I2CMode,
  InterconnectConfig_ServiceVersion: InterconnectConfig_ServiceVersion,
  GPIOState: GPIOState,
  GPIOMode: GPIOMode,
  I2CDeviceIdentification: I2CDeviceIdentification,
  I2CDeviceAddressing: I2CDeviceAddressing,
  InterconnectCyclic_Command: InterconnectCyclic_Command,
  InterconnectCyclic_MessageId: InterconnectCyclic_MessageId,
  InterconnectCyclic_Feedback_tool_feedback: InterconnectCyclic_Feedback_tool_feedback,
  InterconnectCyclic_Feedback: InterconnectCyclic_Feedback,
  InterconnectCyclic_Command_tool_command: InterconnectCyclic_Command_tool_command,
  InterconnectCyclic_ServiceVersion: InterconnectCyclic_ServiceVersion,
  InterconnectCyclic_CustomData: InterconnectCyclic_CustomData,
  InterconnectCyclic_CustomData_tool_customData: InterconnectCyclic_CustomData_tool_customData,
  WristType: WristType,
  ModelId: ModelId,
  ArmLaterality: ArmLaterality,
  ProductConfigurationEndEffectorType: ProductConfigurationEndEffectorType,
  BrakeType: BrakeType,
  BaseType: BaseType,
  CompleteProductConfiguration: CompleteProductConfiguration,
  VisionModuleType: VisionModuleType,
  InterfaceModuleType: InterfaceModuleType,
  EndEffectorType: EndEffectorType,
  BitRate: BitRate,
  SensorIdentifier: SensorIdentifier,
  OptionValue: OptionValue,
  ExtrinsicParameters: ExtrinsicParameters,
  FocusAction: FocusAction,
  SensorFocusAction: SensorFocusAction,
  VisionConfig_RotationMatrixRow: VisionConfig_RotationMatrixRow,
  Resolution: Resolution,
  IntrinsicParameters: IntrinsicParameters,
  VisionNotification: VisionNotification,
  ManualFocus: ManualFocus,
  FrameRate: FrameRate,
  OptionInformation: OptionInformation,
  OptionIdentifier: OptionIdentifier,
  TranslationVector: TranslationVector,
  SensorSettings: SensorSettings,
  SensorFocusAction_action_parameters: SensorFocusAction_action_parameters,
  VisionConfig_RotationMatrix: VisionConfig_RotationMatrix,
  DistortionCoefficients: DistortionCoefficients,
  FocusPoint: FocusPoint,
  Sensor: Sensor,
  VisionEvent: VisionEvent,
  VisionConfig_ServiceVersion: VisionConfig_ServiceVersion,
  IntrinsicProfileIdentifier: IntrinsicProfileIdentifier,
  Option: Option,
  FollowCartesianTrajectoryResult: FollowCartesianTrajectoryResult,
  FollowCartesianTrajectoryFeedback: FollowCartesianTrajectoryFeedback,
  FollowCartesianTrajectoryActionFeedback: FollowCartesianTrajectoryActionFeedback,
  FollowCartesianTrajectoryAction: FollowCartesianTrajectoryAction,
  FollowCartesianTrajectoryActionGoal: FollowCartesianTrajectoryActionGoal,
  FollowCartesianTrajectoryActionResult: FollowCartesianTrajectoryActionResult,
  FollowCartesianTrajectoryGoal: FollowCartesianTrajectoryGoal,
};
