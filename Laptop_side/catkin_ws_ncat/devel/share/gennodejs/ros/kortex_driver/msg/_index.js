
"use strict";

let ApiOptions = require('./ApiOptions.js');
let KortexError = require('./KortexError.js');
let ErrorCodes = require('./ErrorCodes.js');
let SubErrorCodes = require('./SubErrorCodes.js');
let LoopSelection = require('./LoopSelection.js');
let TorqueCalibration = require('./TorqueCalibration.js');
let ControlLoopSelection = require('./ControlLoopSelection.js');
let CommandMode = require('./CommandMode.js');
let TorqueOffset = require('./TorqueOffset.js');
let RampResponse = require('./RampResponse.js');
let CoggingFeedforwardMode = require('./CoggingFeedforwardMode.js');
let PositionCommand = require('./PositionCommand.js');
let CustomDataSelection = require('./CustomDataSelection.js');
let SafetyIdentifierBankA = require('./SafetyIdentifierBankA.js');
let FrequencyResponse = require('./FrequencyResponse.js');
let VectorDriveParameters = require('./VectorDriveParameters.js');
let CoggingFeedforwardModeInformation = require('./CoggingFeedforwardModeInformation.js');
let Servoing = require('./Servoing.js');
let ActuatorConfig_SafetyLimitType = require('./ActuatorConfig_SafetyLimitType.js');
let AxisOffsets = require('./AxisOffsets.js');
let CustomDataIndex = require('./CustomDataIndex.js');
let ActuatorConfig_ControlModeInformation = require('./ActuatorConfig_ControlModeInformation.js');
let ActuatorConfig_ServiceVersion = require('./ActuatorConfig_ServiceVersion.js');
let ActuatorConfig_ControlMode = require('./ActuatorConfig_ControlMode.js');
let EncoderDerivativeParameters = require('./EncoderDerivativeParameters.js');
let AxisPosition = require('./AxisPosition.js');
let CommandModeInformation = require('./CommandModeInformation.js');
let StepResponse = require('./StepResponse.js');
let ControlLoop = require('./ControlLoop.js');
let ControlLoopParameters = require('./ControlLoopParameters.js');
let ActuatorCyclic_MessageId = require('./ActuatorCyclic_MessageId.js');
let ActuatorCyclic_CustomData = require('./ActuatorCyclic_CustomData.js');
let ActuatorCyclic_Feedback = require('./ActuatorCyclic_Feedback.js');
let CommandFlags = require('./CommandFlags.js');
let ActuatorCyclic_Command = require('./ActuatorCyclic_Command.js');
let StatusFlags = require('./StatusFlags.js');
let ActuatorCyclic_ServiceVersion = require('./ActuatorCyclic_ServiceVersion.js');
let Timeout = require('./Timeout.js');
let WifiEnableState = require('./WifiEnableState.js');
let JointTrajectoryConstraint = require('./JointTrajectoryConstraint.js');
let LimitationType = require('./LimitationType.js');
let BridgeStatus = require('./BridgeStatus.js');
let Query = require('./Query.js');
let UserProfile = require('./UserProfile.js');
let GpioAction = require('./GpioAction.js');
let Base_GpioConfiguration = require('./Base_GpioConfiguration.js');
let WifiConfigurationList = require('./WifiConfigurationList.js');
let SafetyEvent = require('./SafetyEvent.js');
let OperatingMode = require('./OperatingMode.js');
let Admittance = require('./Admittance.js');
let ControllerElementEventType = require('./ControllerElementEventType.js');
let JointAngles = require('./JointAngles.js');
let BridgeIdentifier = require('./BridgeIdentifier.js');
let Mapping = require('./Mapping.js');
let SequenceInfoNotification = require('./SequenceInfoNotification.js');
let GripperMode = require('./GripperMode.js');
let GpioCommand = require('./GpioCommand.js');
let TrajectoryInfoType = require('./TrajectoryInfoType.js');
let CartesianTrajectoryConstraint_type = require('./CartesianTrajectoryConstraint_type.js');
let TrajectoryErrorReport = require('./TrajectoryErrorReport.js');
let Twist = require('./Twist.js');
let GpioBehavior = require('./GpioBehavior.js');
let ProtectionZoneEvent = require('./ProtectionZoneEvent.js');
let KinematicTrajectoryConstraints = require('./KinematicTrajectoryConstraints.js');
let ProtectionZoneNotificationList = require('./ProtectionZoneNotificationList.js');
let ArmStateInformation = require('./ArmStateInformation.js');
let RequestedActionType = require('./RequestedActionType.js');
let AppendActionInformation = require('./AppendActionInformation.js');
let PreComputedJointTrajectory = require('./PreComputedJointTrajectory.js');
let MappingHandle = require('./MappingHandle.js');
let MapEvent_events = require('./MapEvent_events.js');
let BluetoothEnableState = require('./BluetoothEnableState.js');
let Base_CapSenseConfig = require('./Base_CapSenseConfig.js');
let JointSpeed = require('./JointSpeed.js');
let Ssid = require('./Ssid.js');
let WrenchMode = require('./WrenchMode.js');
let GpioEvent = require('./GpioEvent.js');
let JointTorques = require('./JointTorques.js');
let Base_JointSpeeds = require('./Base_JointSpeeds.js');
let IPv4Information = require('./IPv4Information.js');
let ActivateMapHandle = require('./ActivateMapHandle.js');
let ArmStateNotification = require('./ArmStateNotification.js');
let ConfigurationChangeNotificationList = require('./ConfigurationChangeNotificationList.js');
let MappingInfoNotificationList = require('./MappingInfoNotificationList.js');
let RobotEvent = require('./RobotEvent.js');
let ActionExecutionState = require('./ActionExecutionState.js');
let AdvancedSequenceHandle = require('./AdvancedSequenceHandle.js');
let Waypoint = require('./Waypoint.js');
let JointLimitation = require('./JointLimitation.js');
let PreComputedJointTrajectoryElement = require('./PreComputedJointTrajectoryElement.js');
let ProtectionZoneNotification = require('./ProtectionZoneNotification.js');
let ControllerConfigurationList = require('./ControllerConfigurationList.js');
let Snapshot = require('./Snapshot.js');
let WrenchCommand = require('./WrenchCommand.js');
let EventIdSequenceInfoNotification = require('./EventIdSequenceInfoNotification.js');
let NetworkEvent = require('./NetworkEvent.js');
let ConfigurationNotificationEvent = require('./ConfigurationNotificationEvent.js');
let MapElement = require('./MapElement.js');
let RobotEventNotificationList = require('./RobotEventNotificationList.js');
let BridgeList = require('./BridgeList.js');
let ControllerConfiguration = require('./ControllerConfiguration.js');
let ControllerElementHandle_identifier = require('./ControllerElementHandle_identifier.js');
let ProtectionZoneList = require('./ProtectionZoneList.js');
let SequenceHandle = require('./SequenceHandle.js');
let ControllerNotification_state = require('./ControllerNotification_state.js');
let ControllerNotification = require('./ControllerNotification.js');
let ActionHandle = require('./ActionHandle.js');
let ConstrainedJointAngle = require('./ConstrainedJointAngle.js');
let SequenceList = require('./SequenceList.js');
let TrajectoryErrorElement = require('./TrajectoryErrorElement.js');
let JointNavigationDirection = require('./JointNavigationDirection.js');
let TrajectoryErrorIdentifier = require('./TrajectoryErrorIdentifier.js');
let SequenceInformation = require('./SequenceInformation.js');
let ControllerElementState = require('./ControllerElementState.js');
let Action = require('./Action.js');
let SystemTime = require('./SystemTime.js');
let GpioConfigurationList = require('./GpioConfigurationList.js');
let Xbox360DigitalInputIdentifier = require('./Xbox360DigitalInputIdentifier.js');
let ConstrainedJointAngles = require('./ConstrainedJointAngles.js');
let MapGroupList = require('./MapGroupList.js');
let BridgePortConfig = require('./BridgePortConfig.js');
let WifiInformation = require('./WifiInformation.js');
let TrajectoryContinuityMode = require('./TrajectoryContinuityMode.js');
let WaypointValidationReport = require('./WaypointValidationReport.js');
let ServoingMode = require('./ServoingMode.js');
let ControllerState = require('./ControllerState.js');
let JointsLimitationsList = require('./JointsLimitationsList.js');
let UserList = require('./UserList.js');
let NetworkNotification = require('./NetworkNotification.js');
let MapList = require('./MapList.js');
let WifiInformationList = require('./WifiInformationList.js');
let SequenceTaskConfiguration = require('./SequenceTaskConfiguration.js');
let ActionEvent = require('./ActionEvent.js');
let UserNotification = require('./UserNotification.js');
let AngularWaypoint = require('./AngularWaypoint.js');
let BridgeResult = require('./BridgeResult.js');
let BridgeType = require('./BridgeType.js');
let ControllerBehavior = require('./ControllerBehavior.js');
let ControllerNotificationList = require('./ControllerNotificationList.js');
let NetworkType = require('./NetworkType.js');
let Base_RotationMatrix = require('./Base_RotationMatrix.js');
let NavigationDirection = require('./NavigationDirection.js');
let Action_action_parameters = require('./Action_action_parameters.js');
let FirmwareComponentVersion = require('./FirmwareComponentVersion.js');
let ChangeTwist = require('./ChangeTwist.js');
let ProtectionZoneHandle = require('./ProtectionZoneHandle.js');
let JointTorque = require('./JointTorque.js');
let ControllerEvent = require('./ControllerEvent.js');
let TwistLimitation = require('./TwistLimitation.js');
let MappingList = require('./MappingList.js');
let OperatingModeInformation = require('./OperatingModeInformation.js');
let GripperRequest = require('./GripperRequest.js');
let SequenceTasksRange = require('./SequenceTasksRange.js');
let FactoryNotification = require('./FactoryNotification.js');
let Pose = require('./Pose.js');
let SoundType = require('./SoundType.js');
let WifiEncryptionType = require('./WifiEncryptionType.js');
let FullUserProfile = require('./FullUserProfile.js');
let UserEvent = require('./UserEvent.js');
let SequenceTasksPair = require('./SequenceTasksPair.js');
let MapGroup = require('./MapGroup.js');
let NetworkHandle = require('./NetworkHandle.js');
let ControlModeNotificationList = require('./ControlModeNotificationList.js');
let LedState = require('./LedState.js');
let PasswordChange = require('./PasswordChange.js');
let ControllerEventType = require('./ControllerEventType.js');
let ControllerElementHandle = require('./ControllerElementHandle.js');
let WristDigitalInputIdentifier = require('./WristDigitalInputIdentifier.js');
let SafetyNotificationList = require('./SafetyNotificationList.js');
let CartesianWaypoint = require('./CartesianWaypoint.js');
let MapEvent = require('./MapEvent.js');
let SnapshotType = require('./SnapshotType.js');
let Base_ControlMode = require('./Base_ControlMode.js');
let ActionList = require('./ActionList.js');
let Finger = require('./Finger.js');
let ConfigurationChangeNotification_configuration_change = require('./ConfigurationChangeNotification_configuration_change.js');
let ServoingModeNotificationList = require('./ServoingModeNotificationList.js');
let ControllerType = require('./ControllerType.js');
let WifiSecurityType = require('./WifiSecurityType.js');
let TrajectoryInfo = require('./TrajectoryInfo.js');
let MapGroupHandle = require('./MapGroupHandle.js');
let SequenceTask = require('./SequenceTask.js');
let CartesianSpeed = require('./CartesianSpeed.js');
let IKData = require('./IKData.js');
let IPv4Configuration = require('./IPv4Configuration.js');
let FactoryEvent = require('./FactoryEvent.js');
let ActionType = require('./ActionType.js');
let ControllerInputType = require('./ControllerInputType.js');
let Xbox360AnalogInputIdentifier = require('./Xbox360AnalogInputIdentifier.js');
let ZoneShape = require('./ZoneShape.js');
let WifiConfiguration = require('./WifiConfiguration.js');
let WaypointList = require('./WaypointList.js');
let OperatingModeNotification = require('./OperatingModeNotification.js');
let Gen3GpioPinId = require('./Gen3GpioPinId.js');
let CommunicationInterfaceConfiguration = require('./CommunicationInterfaceConfiguration.js');
let ProtectionZone = require('./ProtectionZone.js');
let Base_ServiceVersion = require('./Base_ServiceVersion.js');
let Base_ControlModeInformation = require('./Base_ControlModeInformation.js');
let ServoingModeInformation = require('./ServoingModeInformation.js');
let JointTrajectoryConstraintType = require('./JointTrajectoryConstraintType.js');
let MapHandle = require('./MapHandle.js');
let OperatingModeNotificationList = require('./OperatingModeNotificationList.js');
let RobotEventNotification = require('./RobotEventNotification.js');
let TransformationMatrix = require('./TransformationMatrix.js');
let CartesianLimitationList = require('./CartesianLimitationList.js');
let JointAngle = require('./JointAngle.js');
let FullIPv4Configuration = require('./FullIPv4Configuration.js');
let ActionNotificationList = require('./ActionNotificationList.js');
let WrenchLimitation = require('./WrenchLimitation.js');
let ShapeType = require('./ShapeType.js');
let TrajectoryErrorType = require('./TrajectoryErrorType.js');
let NetworkNotificationList = require('./NetworkNotificationList.js');
let ProtectionZoneInformation = require('./ProtectionZoneInformation.js');
let ServoingModeNotification = require('./ServoingModeNotification.js');
let SequenceTasks = require('./SequenceTasks.js');
let Base_ControlModeNotification = require('./Base_ControlModeNotification.js');
let ConstrainedOrientation = require('./ConstrainedOrientation.js');
let EmergencyStop = require('./EmergencyStop.js');
let AdmittanceMode = require('./AdmittanceMode.js');
let SequenceInfoNotificationList = require('./SequenceInfoNotificationList.js');
let Base_CapSenseMode = require('./Base_CapSenseMode.js');
let UserProfileList = require('./UserProfileList.js');
let CartesianLimitation = require('./CartesianLimitation.js');
let BridgeConfig = require('./BridgeConfig.js');
let CartesianTrajectoryConstraint = require('./CartesianTrajectoryConstraint.js');
let ActuatorInformation = require('./ActuatorInformation.js');
let ChangeJointSpeeds = require('./ChangeJointSpeeds.js');
let ControllerHandle = require('./ControllerHandle.js');
let SwitchControlMapping = require('./SwitchControlMapping.js');
let SequenceTaskHandle = require('./SequenceTaskHandle.js');
let Map = require('./Map.js');
let RFConfiguration = require('./RFConfiguration.js');
let Wrench = require('./Wrench.js');
let ConfigurationChangeNotification = require('./ConfigurationChangeNotification.js');
let ControllerConfigurationMode = require('./ControllerConfigurationMode.js');
let BackupEvent = require('./BackupEvent.js');
let Waypoint_type_of_waypoint = require('./Waypoint_type_of_waypoint.js');
let FirmwareBundleVersions = require('./FirmwareBundleVersions.js');
let TwistCommand = require('./TwistCommand.js');
let SequenceTasksConfiguration = require('./SequenceTasksConfiguration.js');
let GpioPinPropertyFlags = require('./GpioPinPropertyFlags.js');
let ConstrainedPosition = require('./ConstrainedPosition.js');
let Orientation = require('./Orientation.js');
let TransformationRow = require('./TransformationRow.js');
let GpioPinConfiguration = require('./GpioPinConfiguration.js');
let Faults = require('./Faults.js');
let GripperCommand = require('./GripperCommand.js');
let Base_RotationMatrixRow = require('./Base_RotationMatrixRow.js');
let Base_SafetyIdentifier = require('./Base_SafetyIdentifier.js');
let SignalQuality = require('./SignalQuality.js');
let ChangeWrench = require('./ChangeWrench.js');
let MappingInfoNotification = require('./MappingInfoNotification.js');
let Delay = require('./Delay.js');
let Sequence = require('./Sequence.js');
let Base_Position = require('./Base_Position.js');
let Point = require('./Point.js');
let UserNotificationList = require('./UserNotificationList.js');
let ControllerList = require('./ControllerList.js');
let Base_Stop = require('./Base_Stop.js');
let ConstrainedPose = require('./ConstrainedPose.js');
let ActionNotification = require('./ActionNotification.js');
let Gripper = require('./Gripper.js');
let ActuatorFeedback = require('./ActuatorFeedback.js');
let BaseCyclic_ServiceVersion = require('./BaseCyclic_ServiceVersion.js');
let ActuatorCommand = require('./ActuatorCommand.js');
let BaseFeedback = require('./BaseFeedback.js');
let BaseCyclic_Feedback = require('./BaseCyclic_Feedback.js');
let BaseCyclic_Command = require('./BaseCyclic_Command.js');
let BaseCyclic_CustomData = require('./BaseCyclic_CustomData.js');
let ActuatorCustomData = require('./ActuatorCustomData.js');
let UARTSpeed = require('./UARTSpeed.js');
let CountryCodeIdentifier = require('./CountryCodeIdentifier.js');
let UserProfileHandle = require('./UserProfileHandle.js');
let UARTDeviceIdentification = require('./UARTDeviceIdentification.js');
let UARTParity = require('./UARTParity.js');
let CartesianReferenceFrame = require('./CartesianReferenceFrame.js');
let ArmState = require('./ArmState.js');
let SafetyNotification = require('./SafetyNotification.js');
let SafetyHandle = require('./SafetyHandle.js');
let CountryCode = require('./CountryCode.js');
let NotificationOptions = require('./NotificationOptions.js');
let Empty = require('./Empty.js');
let UARTWordLength = require('./UARTWordLength.js');
let SafetyStatusValue = require('./SafetyStatusValue.js');
let UARTConfiguration = require('./UARTConfiguration.js');
let Timestamp = require('./Timestamp.js');
let NotificationHandle = require('./NotificationHandle.js');
let Permission = require('./Permission.js');
let UARTStopBits = require('./UARTStopBits.js');
let DeviceTypes = require('./DeviceTypes.js');
let Unit = require('./Unit.js');
let Connection = require('./Connection.js');
let NotificationType = require('./NotificationType.js');
let DeviceHandle = require('./DeviceHandle.js');
let DesiredSpeeds = require('./DesiredSpeeds.js');
let ControlConfig_ControlMode = require('./ControlConfig_ControlMode.js');
let ControlConfigurationNotification = require('./ControlConfigurationNotification.js');
let CartesianReferenceFrameInfo = require('./CartesianReferenceFrameInfo.js');
let JointSpeedSoftLimits = require('./JointSpeedSoftLimits.js');
let LinearTwist = require('./LinearTwist.js');
let KinematicLimits = require('./KinematicLimits.js');
let ControlConfig_ControlModeNotification = require('./ControlConfig_ControlModeNotification.js');
let ControlConfig_Position = require('./ControlConfig_Position.js');
let JointAccelerationSoftLimits = require('./JointAccelerationSoftLimits.js');
let TwistAngularSoftLimit = require('./TwistAngularSoftLimit.js');
let PayloadInformation = require('./PayloadInformation.js');
let ControlConfig_ServiceVersion = require('./ControlConfig_ServiceVersion.js');
let ControlConfig_ControlModeInformation = require('./ControlConfig_ControlModeInformation.js');
let ControlConfig_JointSpeeds = require('./ControlConfig_JointSpeeds.js');
let AngularTwist = require('./AngularTwist.js');
let TwistLinearSoftLimit = require('./TwistLinearSoftLimit.js');
let GravityVector = require('./GravityVector.js');
let CartesianTransform = require('./CartesianTransform.js');
let KinematicLimitsList = require('./KinematicLimitsList.js');
let ToolConfiguration = require('./ToolConfiguration.js');
let ControlConfigurationEvent = require('./ControlConfigurationEvent.js');
let RunMode = require('./RunMode.js');
let CalibrationResult = require('./CalibrationResult.js');
let IPv4Settings = require('./IPv4Settings.js');
let DeviceType = require('./DeviceType.js');
let ModelNumber = require('./ModelNumber.js');
let CalibrationItem = require('./CalibrationItem.js');
let RebootRqst = require('./RebootRqst.js');
let FirmwareVersion = require('./FirmwareVersion.js');
let SafetyThreshold = require('./SafetyThreshold.js');
let PartNumber = require('./PartNumber.js');
let DeviceConfig_SafetyLimitType = require('./DeviceConfig_SafetyLimitType.js');
let CalibrationElement = require('./CalibrationElement.js');
let BootloaderVersion = require('./BootloaderVersion.js');
let SafetyConfigurationList = require('./SafetyConfigurationList.js');
let SafetyInformationList = require('./SafetyInformationList.js');
let SafetyConfiguration = require('./SafetyConfiguration.js');
let PartNumberRevision = require('./PartNumberRevision.js');
let MACAddress = require('./MACAddress.js');
let Calibration = require('./Calibration.js');
let CalibrationParameter = require('./CalibrationParameter.js');
let SerialNumber = require('./SerialNumber.js');
let CapSenseRegister = require('./CapSenseRegister.js');
let PowerOnSelfTestResult = require('./PowerOnSelfTestResult.js');
let CalibrationParameter_value = require('./CalibrationParameter_value.js');
let DeviceConfig_ServiceVersion = require('./DeviceConfig_ServiceVersion.js');
let DeviceConfig_CapSenseMode = require('./DeviceConfig_CapSenseMode.js');
let SafetyEnable = require('./SafetyEnable.js');
let SafetyInformation = require('./SafetyInformation.js');
let DeviceConfig_CapSenseConfig = require('./DeviceConfig_CapSenseConfig.js');
let CalibrationStatus = require('./CalibrationStatus.js');
let SafetyStatus = require('./SafetyStatus.js');
let RunModes = require('./RunModes.js');
let DeviceManager_ServiceVersion = require('./DeviceManager_ServiceVersion.js');
let DeviceHandles = require('./DeviceHandles.js');
let RobotiqGripperStatusFlags = require('./RobotiqGripperStatusFlags.js');
let GripperConfig_SafetyIdentifier = require('./GripperConfig_SafetyIdentifier.js');
let GripperCyclic_CustomData = require('./GripperCyclic_CustomData.js');
let MotorFeedback = require('./MotorFeedback.js');
let MotorCommand = require('./MotorCommand.js');
let GripperCyclic_Command = require('./GripperCyclic_Command.js');
let GripperCyclic_ServiceVersion = require('./GripperCyclic_ServiceVersion.js');
let GripperCyclic_MessageId = require('./GripperCyclic_MessageId.js');
let CustomDataUnit = require('./CustomDataUnit.js');
let GripperCyclic_Feedback = require('./GripperCyclic_Feedback.js');
let GPIOIdentification = require('./GPIOIdentification.js');
let UARTPortId = require('./UARTPortId.js');
let EthernetDevice = require('./EthernetDevice.js');
let InterconnectConfig_SafetyIdentifier = require('./InterconnectConfig_SafetyIdentifier.js');
let EthernetSpeed = require('./EthernetSpeed.js');
let I2CReadParameter = require('./I2CReadParameter.js');
let I2CWriteParameter = require('./I2CWriteParameter.js');
let InterconnectConfig_ServiceVersion = require('./InterconnectConfig_ServiceVersion.js');
let GPIOValue = require('./GPIOValue.js');
let I2CDevice = require('./I2CDevice.js');
let I2CRegisterAddressSize = require('./I2CRegisterAddressSize.js');
let I2CWriteRegisterParameter = require('./I2CWriteRegisterParameter.js');
let EthernetDuplex = require('./EthernetDuplex.js');
let I2CConfiguration = require('./I2CConfiguration.js');
let GPIOState = require('./GPIOState.js');
let I2CReadRegisterParameter = require('./I2CReadRegisterParameter.js');
let EthernetConfiguration = require('./EthernetConfiguration.js');
let I2CData = require('./I2CData.js');
let I2CDeviceIdentification = require('./I2CDeviceIdentification.js');
let GPIOPull = require('./GPIOPull.js');
let EthernetDeviceIdentification = require('./EthernetDeviceIdentification.js');
let InterconnectConfig_GPIOConfiguration = require('./InterconnectConfig_GPIOConfiguration.js');
let GPIOMode = require('./GPIOMode.js');
let GPIOIdentifier = require('./GPIOIdentifier.js');
let I2CMode = require('./I2CMode.js');
let I2CDeviceAddressing = require('./I2CDeviceAddressing.js');
let InterconnectCyclic_Command_tool_command = require('./InterconnectCyclic_Command_tool_command.js');
let InterconnectCyclic_CustomData = require('./InterconnectCyclic_CustomData.js');
let InterconnectCyclic_Feedback = require('./InterconnectCyclic_Feedback.js');
let InterconnectCyclic_MessageId = require('./InterconnectCyclic_MessageId.js');
let InterconnectCyclic_ServiceVersion = require('./InterconnectCyclic_ServiceVersion.js');
let InterconnectCyclic_Feedback_tool_feedback = require('./InterconnectCyclic_Feedback_tool_feedback.js');
let InterconnectCyclic_CustomData_tool_customData = require('./InterconnectCyclic_CustomData_tool_customData.js');
let InterconnectCyclic_Command = require('./InterconnectCyclic_Command.js');
let CompleteProductConfiguration = require('./CompleteProductConfiguration.js');
let BrakeType = require('./BrakeType.js');
let ProductConfigurationEndEffectorType = require('./ProductConfigurationEndEffectorType.js');
let WristType = require('./WristType.js');
let EndEffectorType = require('./EndEffectorType.js');
let VisionModuleType = require('./VisionModuleType.js');
let InterfaceModuleType = require('./InterfaceModuleType.js');
let BaseType = require('./BaseType.js');
let ModelId = require('./ModelId.js');
let ArmLaterality = require('./ArmLaterality.js');
let IntrinsicProfileIdentifier = require('./IntrinsicProfileIdentifier.js');
let OptionInformation = require('./OptionInformation.js');
let SensorFocusAction_action_parameters = require('./SensorFocusAction_action_parameters.js');
let DistortionCoefficients = require('./DistortionCoefficients.js');
let ManualFocus = require('./ManualFocus.js');
let VisionEvent = require('./VisionEvent.js');
let OptionValue = require('./OptionValue.js');
let SensorSettings = require('./SensorSettings.js');
let FocusPoint = require('./FocusPoint.js');
let TranslationVector = require('./TranslationVector.js');
let ExtrinsicParameters = require('./ExtrinsicParameters.js');
let VisionConfig_ServiceVersion = require('./VisionConfig_ServiceVersion.js');
let FrameRate = require('./FrameRate.js');
let Resolution = require('./Resolution.js');
let FocusAction = require('./FocusAction.js');
let SensorFocusAction = require('./SensorFocusAction.js');
let IntrinsicParameters = require('./IntrinsicParameters.js');
let OptionIdentifier = require('./OptionIdentifier.js');
let SensorIdentifier = require('./SensorIdentifier.js');
let Sensor = require('./Sensor.js');
let VisionConfig_RotationMatrixRow = require('./VisionConfig_RotationMatrixRow.js');
let BitRate = require('./BitRate.js');
let VisionNotification = require('./VisionNotification.js');
let Option = require('./Option.js');
let VisionConfig_RotationMatrix = require('./VisionConfig_RotationMatrix.js');
let FollowCartesianTrajectoryActionGoal = require('./FollowCartesianTrajectoryActionGoal.js');
let FollowCartesianTrajectoryActionResult = require('./FollowCartesianTrajectoryActionResult.js');
let FollowCartesianTrajectoryGoal = require('./FollowCartesianTrajectoryGoal.js');
let FollowCartesianTrajectoryResult = require('./FollowCartesianTrajectoryResult.js');
let FollowCartesianTrajectoryAction = require('./FollowCartesianTrajectoryAction.js');
let FollowCartesianTrajectoryFeedback = require('./FollowCartesianTrajectoryFeedback.js');
let FollowCartesianTrajectoryActionFeedback = require('./FollowCartesianTrajectoryActionFeedback.js');

module.exports = {
  ApiOptions: ApiOptions,
  KortexError: KortexError,
  ErrorCodes: ErrorCodes,
  SubErrorCodes: SubErrorCodes,
  LoopSelection: LoopSelection,
  TorqueCalibration: TorqueCalibration,
  ControlLoopSelection: ControlLoopSelection,
  CommandMode: CommandMode,
  TorqueOffset: TorqueOffset,
  RampResponse: RampResponse,
  CoggingFeedforwardMode: CoggingFeedforwardMode,
  PositionCommand: PositionCommand,
  CustomDataSelection: CustomDataSelection,
  SafetyIdentifierBankA: SafetyIdentifierBankA,
  FrequencyResponse: FrequencyResponse,
  VectorDriveParameters: VectorDriveParameters,
  CoggingFeedforwardModeInformation: CoggingFeedforwardModeInformation,
  Servoing: Servoing,
  ActuatorConfig_SafetyLimitType: ActuatorConfig_SafetyLimitType,
  AxisOffsets: AxisOffsets,
  CustomDataIndex: CustomDataIndex,
  ActuatorConfig_ControlModeInformation: ActuatorConfig_ControlModeInformation,
  ActuatorConfig_ServiceVersion: ActuatorConfig_ServiceVersion,
  ActuatorConfig_ControlMode: ActuatorConfig_ControlMode,
  EncoderDerivativeParameters: EncoderDerivativeParameters,
  AxisPosition: AxisPosition,
  CommandModeInformation: CommandModeInformation,
  StepResponse: StepResponse,
  ControlLoop: ControlLoop,
  ControlLoopParameters: ControlLoopParameters,
  ActuatorCyclic_MessageId: ActuatorCyclic_MessageId,
  ActuatorCyclic_CustomData: ActuatorCyclic_CustomData,
  ActuatorCyclic_Feedback: ActuatorCyclic_Feedback,
  CommandFlags: CommandFlags,
  ActuatorCyclic_Command: ActuatorCyclic_Command,
  StatusFlags: StatusFlags,
  ActuatorCyclic_ServiceVersion: ActuatorCyclic_ServiceVersion,
  Timeout: Timeout,
  WifiEnableState: WifiEnableState,
  JointTrajectoryConstraint: JointTrajectoryConstraint,
  LimitationType: LimitationType,
  BridgeStatus: BridgeStatus,
  Query: Query,
  UserProfile: UserProfile,
  GpioAction: GpioAction,
  Base_GpioConfiguration: Base_GpioConfiguration,
  WifiConfigurationList: WifiConfigurationList,
  SafetyEvent: SafetyEvent,
  OperatingMode: OperatingMode,
  Admittance: Admittance,
  ControllerElementEventType: ControllerElementEventType,
  JointAngles: JointAngles,
  BridgeIdentifier: BridgeIdentifier,
  Mapping: Mapping,
  SequenceInfoNotification: SequenceInfoNotification,
  GripperMode: GripperMode,
  GpioCommand: GpioCommand,
  TrajectoryInfoType: TrajectoryInfoType,
  CartesianTrajectoryConstraint_type: CartesianTrajectoryConstraint_type,
  TrajectoryErrorReport: TrajectoryErrorReport,
  Twist: Twist,
  GpioBehavior: GpioBehavior,
  ProtectionZoneEvent: ProtectionZoneEvent,
  KinematicTrajectoryConstraints: KinematicTrajectoryConstraints,
  ProtectionZoneNotificationList: ProtectionZoneNotificationList,
  ArmStateInformation: ArmStateInformation,
  RequestedActionType: RequestedActionType,
  AppendActionInformation: AppendActionInformation,
  PreComputedJointTrajectory: PreComputedJointTrajectory,
  MappingHandle: MappingHandle,
  MapEvent_events: MapEvent_events,
  BluetoothEnableState: BluetoothEnableState,
  Base_CapSenseConfig: Base_CapSenseConfig,
  JointSpeed: JointSpeed,
  Ssid: Ssid,
  WrenchMode: WrenchMode,
  GpioEvent: GpioEvent,
  JointTorques: JointTorques,
  Base_JointSpeeds: Base_JointSpeeds,
  IPv4Information: IPv4Information,
  ActivateMapHandle: ActivateMapHandle,
  ArmStateNotification: ArmStateNotification,
  ConfigurationChangeNotificationList: ConfigurationChangeNotificationList,
  MappingInfoNotificationList: MappingInfoNotificationList,
  RobotEvent: RobotEvent,
  ActionExecutionState: ActionExecutionState,
  AdvancedSequenceHandle: AdvancedSequenceHandle,
  Waypoint: Waypoint,
  JointLimitation: JointLimitation,
  PreComputedJointTrajectoryElement: PreComputedJointTrajectoryElement,
  ProtectionZoneNotification: ProtectionZoneNotification,
  ControllerConfigurationList: ControllerConfigurationList,
  Snapshot: Snapshot,
  WrenchCommand: WrenchCommand,
  EventIdSequenceInfoNotification: EventIdSequenceInfoNotification,
  NetworkEvent: NetworkEvent,
  ConfigurationNotificationEvent: ConfigurationNotificationEvent,
  MapElement: MapElement,
  RobotEventNotificationList: RobotEventNotificationList,
  BridgeList: BridgeList,
  ControllerConfiguration: ControllerConfiguration,
  ControllerElementHandle_identifier: ControllerElementHandle_identifier,
  ProtectionZoneList: ProtectionZoneList,
  SequenceHandle: SequenceHandle,
  ControllerNotification_state: ControllerNotification_state,
  ControllerNotification: ControllerNotification,
  ActionHandle: ActionHandle,
  ConstrainedJointAngle: ConstrainedJointAngle,
  SequenceList: SequenceList,
  TrajectoryErrorElement: TrajectoryErrorElement,
  JointNavigationDirection: JointNavigationDirection,
  TrajectoryErrorIdentifier: TrajectoryErrorIdentifier,
  SequenceInformation: SequenceInformation,
  ControllerElementState: ControllerElementState,
  Action: Action,
  SystemTime: SystemTime,
  GpioConfigurationList: GpioConfigurationList,
  Xbox360DigitalInputIdentifier: Xbox360DigitalInputIdentifier,
  ConstrainedJointAngles: ConstrainedJointAngles,
  MapGroupList: MapGroupList,
  BridgePortConfig: BridgePortConfig,
  WifiInformation: WifiInformation,
  TrajectoryContinuityMode: TrajectoryContinuityMode,
  WaypointValidationReport: WaypointValidationReport,
  ServoingMode: ServoingMode,
  ControllerState: ControllerState,
  JointsLimitationsList: JointsLimitationsList,
  UserList: UserList,
  NetworkNotification: NetworkNotification,
  MapList: MapList,
  WifiInformationList: WifiInformationList,
  SequenceTaskConfiguration: SequenceTaskConfiguration,
  ActionEvent: ActionEvent,
  UserNotification: UserNotification,
  AngularWaypoint: AngularWaypoint,
  BridgeResult: BridgeResult,
  BridgeType: BridgeType,
  ControllerBehavior: ControllerBehavior,
  ControllerNotificationList: ControllerNotificationList,
  NetworkType: NetworkType,
  Base_RotationMatrix: Base_RotationMatrix,
  NavigationDirection: NavigationDirection,
  Action_action_parameters: Action_action_parameters,
  FirmwareComponentVersion: FirmwareComponentVersion,
  ChangeTwist: ChangeTwist,
  ProtectionZoneHandle: ProtectionZoneHandle,
  JointTorque: JointTorque,
  ControllerEvent: ControllerEvent,
  TwistLimitation: TwistLimitation,
  MappingList: MappingList,
  OperatingModeInformation: OperatingModeInformation,
  GripperRequest: GripperRequest,
  SequenceTasksRange: SequenceTasksRange,
  FactoryNotification: FactoryNotification,
  Pose: Pose,
  SoundType: SoundType,
  WifiEncryptionType: WifiEncryptionType,
  FullUserProfile: FullUserProfile,
  UserEvent: UserEvent,
  SequenceTasksPair: SequenceTasksPair,
  MapGroup: MapGroup,
  NetworkHandle: NetworkHandle,
  ControlModeNotificationList: ControlModeNotificationList,
  LedState: LedState,
  PasswordChange: PasswordChange,
  ControllerEventType: ControllerEventType,
  ControllerElementHandle: ControllerElementHandle,
  WristDigitalInputIdentifier: WristDigitalInputIdentifier,
  SafetyNotificationList: SafetyNotificationList,
  CartesianWaypoint: CartesianWaypoint,
  MapEvent: MapEvent,
  SnapshotType: SnapshotType,
  Base_ControlMode: Base_ControlMode,
  ActionList: ActionList,
  Finger: Finger,
  ConfigurationChangeNotification_configuration_change: ConfigurationChangeNotification_configuration_change,
  ServoingModeNotificationList: ServoingModeNotificationList,
  ControllerType: ControllerType,
  WifiSecurityType: WifiSecurityType,
  TrajectoryInfo: TrajectoryInfo,
  MapGroupHandle: MapGroupHandle,
  SequenceTask: SequenceTask,
  CartesianSpeed: CartesianSpeed,
  IKData: IKData,
  IPv4Configuration: IPv4Configuration,
  FactoryEvent: FactoryEvent,
  ActionType: ActionType,
  ControllerInputType: ControllerInputType,
  Xbox360AnalogInputIdentifier: Xbox360AnalogInputIdentifier,
  ZoneShape: ZoneShape,
  WifiConfiguration: WifiConfiguration,
  WaypointList: WaypointList,
  OperatingModeNotification: OperatingModeNotification,
  Gen3GpioPinId: Gen3GpioPinId,
  CommunicationInterfaceConfiguration: CommunicationInterfaceConfiguration,
  ProtectionZone: ProtectionZone,
  Base_ServiceVersion: Base_ServiceVersion,
  Base_ControlModeInformation: Base_ControlModeInformation,
  ServoingModeInformation: ServoingModeInformation,
  JointTrajectoryConstraintType: JointTrajectoryConstraintType,
  MapHandle: MapHandle,
  OperatingModeNotificationList: OperatingModeNotificationList,
  RobotEventNotification: RobotEventNotification,
  TransformationMatrix: TransformationMatrix,
  CartesianLimitationList: CartesianLimitationList,
  JointAngle: JointAngle,
  FullIPv4Configuration: FullIPv4Configuration,
  ActionNotificationList: ActionNotificationList,
  WrenchLimitation: WrenchLimitation,
  ShapeType: ShapeType,
  TrajectoryErrorType: TrajectoryErrorType,
  NetworkNotificationList: NetworkNotificationList,
  ProtectionZoneInformation: ProtectionZoneInformation,
  ServoingModeNotification: ServoingModeNotification,
  SequenceTasks: SequenceTasks,
  Base_ControlModeNotification: Base_ControlModeNotification,
  ConstrainedOrientation: ConstrainedOrientation,
  EmergencyStop: EmergencyStop,
  AdmittanceMode: AdmittanceMode,
  SequenceInfoNotificationList: SequenceInfoNotificationList,
  Base_CapSenseMode: Base_CapSenseMode,
  UserProfileList: UserProfileList,
  CartesianLimitation: CartesianLimitation,
  BridgeConfig: BridgeConfig,
  CartesianTrajectoryConstraint: CartesianTrajectoryConstraint,
  ActuatorInformation: ActuatorInformation,
  ChangeJointSpeeds: ChangeJointSpeeds,
  ControllerHandle: ControllerHandle,
  SwitchControlMapping: SwitchControlMapping,
  SequenceTaskHandle: SequenceTaskHandle,
  Map: Map,
  RFConfiguration: RFConfiguration,
  Wrench: Wrench,
  ConfigurationChangeNotification: ConfigurationChangeNotification,
  ControllerConfigurationMode: ControllerConfigurationMode,
  BackupEvent: BackupEvent,
  Waypoint_type_of_waypoint: Waypoint_type_of_waypoint,
  FirmwareBundleVersions: FirmwareBundleVersions,
  TwistCommand: TwistCommand,
  SequenceTasksConfiguration: SequenceTasksConfiguration,
  GpioPinPropertyFlags: GpioPinPropertyFlags,
  ConstrainedPosition: ConstrainedPosition,
  Orientation: Orientation,
  TransformationRow: TransformationRow,
  GpioPinConfiguration: GpioPinConfiguration,
  Faults: Faults,
  GripperCommand: GripperCommand,
  Base_RotationMatrixRow: Base_RotationMatrixRow,
  Base_SafetyIdentifier: Base_SafetyIdentifier,
  SignalQuality: SignalQuality,
  ChangeWrench: ChangeWrench,
  MappingInfoNotification: MappingInfoNotification,
  Delay: Delay,
  Sequence: Sequence,
  Base_Position: Base_Position,
  Point: Point,
  UserNotificationList: UserNotificationList,
  ControllerList: ControllerList,
  Base_Stop: Base_Stop,
  ConstrainedPose: ConstrainedPose,
  ActionNotification: ActionNotification,
  Gripper: Gripper,
  ActuatorFeedback: ActuatorFeedback,
  BaseCyclic_ServiceVersion: BaseCyclic_ServiceVersion,
  ActuatorCommand: ActuatorCommand,
  BaseFeedback: BaseFeedback,
  BaseCyclic_Feedback: BaseCyclic_Feedback,
  BaseCyclic_Command: BaseCyclic_Command,
  BaseCyclic_CustomData: BaseCyclic_CustomData,
  ActuatorCustomData: ActuatorCustomData,
  UARTSpeed: UARTSpeed,
  CountryCodeIdentifier: CountryCodeIdentifier,
  UserProfileHandle: UserProfileHandle,
  UARTDeviceIdentification: UARTDeviceIdentification,
  UARTParity: UARTParity,
  CartesianReferenceFrame: CartesianReferenceFrame,
  ArmState: ArmState,
  SafetyNotification: SafetyNotification,
  SafetyHandle: SafetyHandle,
  CountryCode: CountryCode,
  NotificationOptions: NotificationOptions,
  Empty: Empty,
  UARTWordLength: UARTWordLength,
  SafetyStatusValue: SafetyStatusValue,
  UARTConfiguration: UARTConfiguration,
  Timestamp: Timestamp,
  NotificationHandle: NotificationHandle,
  Permission: Permission,
  UARTStopBits: UARTStopBits,
  DeviceTypes: DeviceTypes,
  Unit: Unit,
  Connection: Connection,
  NotificationType: NotificationType,
  DeviceHandle: DeviceHandle,
  DesiredSpeeds: DesiredSpeeds,
  ControlConfig_ControlMode: ControlConfig_ControlMode,
  ControlConfigurationNotification: ControlConfigurationNotification,
  CartesianReferenceFrameInfo: CartesianReferenceFrameInfo,
  JointSpeedSoftLimits: JointSpeedSoftLimits,
  LinearTwist: LinearTwist,
  KinematicLimits: KinematicLimits,
  ControlConfig_ControlModeNotification: ControlConfig_ControlModeNotification,
  ControlConfig_Position: ControlConfig_Position,
  JointAccelerationSoftLimits: JointAccelerationSoftLimits,
  TwistAngularSoftLimit: TwistAngularSoftLimit,
  PayloadInformation: PayloadInformation,
  ControlConfig_ServiceVersion: ControlConfig_ServiceVersion,
  ControlConfig_ControlModeInformation: ControlConfig_ControlModeInformation,
  ControlConfig_JointSpeeds: ControlConfig_JointSpeeds,
  AngularTwist: AngularTwist,
  TwistLinearSoftLimit: TwistLinearSoftLimit,
  GravityVector: GravityVector,
  CartesianTransform: CartesianTransform,
  KinematicLimitsList: KinematicLimitsList,
  ToolConfiguration: ToolConfiguration,
  ControlConfigurationEvent: ControlConfigurationEvent,
  RunMode: RunMode,
  CalibrationResult: CalibrationResult,
  IPv4Settings: IPv4Settings,
  DeviceType: DeviceType,
  ModelNumber: ModelNumber,
  CalibrationItem: CalibrationItem,
  RebootRqst: RebootRqst,
  FirmwareVersion: FirmwareVersion,
  SafetyThreshold: SafetyThreshold,
  PartNumber: PartNumber,
  DeviceConfig_SafetyLimitType: DeviceConfig_SafetyLimitType,
  CalibrationElement: CalibrationElement,
  BootloaderVersion: BootloaderVersion,
  SafetyConfigurationList: SafetyConfigurationList,
  SafetyInformationList: SafetyInformationList,
  SafetyConfiguration: SafetyConfiguration,
  PartNumberRevision: PartNumberRevision,
  MACAddress: MACAddress,
  Calibration: Calibration,
  CalibrationParameter: CalibrationParameter,
  SerialNumber: SerialNumber,
  CapSenseRegister: CapSenseRegister,
  PowerOnSelfTestResult: PowerOnSelfTestResult,
  CalibrationParameter_value: CalibrationParameter_value,
  DeviceConfig_ServiceVersion: DeviceConfig_ServiceVersion,
  DeviceConfig_CapSenseMode: DeviceConfig_CapSenseMode,
  SafetyEnable: SafetyEnable,
  SafetyInformation: SafetyInformation,
  DeviceConfig_CapSenseConfig: DeviceConfig_CapSenseConfig,
  CalibrationStatus: CalibrationStatus,
  SafetyStatus: SafetyStatus,
  RunModes: RunModes,
  DeviceManager_ServiceVersion: DeviceManager_ServiceVersion,
  DeviceHandles: DeviceHandles,
  RobotiqGripperStatusFlags: RobotiqGripperStatusFlags,
  GripperConfig_SafetyIdentifier: GripperConfig_SafetyIdentifier,
  GripperCyclic_CustomData: GripperCyclic_CustomData,
  MotorFeedback: MotorFeedback,
  MotorCommand: MotorCommand,
  GripperCyclic_Command: GripperCyclic_Command,
  GripperCyclic_ServiceVersion: GripperCyclic_ServiceVersion,
  GripperCyclic_MessageId: GripperCyclic_MessageId,
  CustomDataUnit: CustomDataUnit,
  GripperCyclic_Feedback: GripperCyclic_Feedback,
  GPIOIdentification: GPIOIdentification,
  UARTPortId: UARTPortId,
  EthernetDevice: EthernetDevice,
  InterconnectConfig_SafetyIdentifier: InterconnectConfig_SafetyIdentifier,
  EthernetSpeed: EthernetSpeed,
  I2CReadParameter: I2CReadParameter,
  I2CWriteParameter: I2CWriteParameter,
  InterconnectConfig_ServiceVersion: InterconnectConfig_ServiceVersion,
  GPIOValue: GPIOValue,
  I2CDevice: I2CDevice,
  I2CRegisterAddressSize: I2CRegisterAddressSize,
  I2CWriteRegisterParameter: I2CWriteRegisterParameter,
  EthernetDuplex: EthernetDuplex,
  I2CConfiguration: I2CConfiguration,
  GPIOState: GPIOState,
  I2CReadRegisterParameter: I2CReadRegisterParameter,
  EthernetConfiguration: EthernetConfiguration,
  I2CData: I2CData,
  I2CDeviceIdentification: I2CDeviceIdentification,
  GPIOPull: GPIOPull,
  EthernetDeviceIdentification: EthernetDeviceIdentification,
  InterconnectConfig_GPIOConfiguration: InterconnectConfig_GPIOConfiguration,
  GPIOMode: GPIOMode,
  GPIOIdentifier: GPIOIdentifier,
  I2CMode: I2CMode,
  I2CDeviceAddressing: I2CDeviceAddressing,
  InterconnectCyclic_Command_tool_command: InterconnectCyclic_Command_tool_command,
  InterconnectCyclic_CustomData: InterconnectCyclic_CustomData,
  InterconnectCyclic_Feedback: InterconnectCyclic_Feedback,
  InterconnectCyclic_MessageId: InterconnectCyclic_MessageId,
  InterconnectCyclic_ServiceVersion: InterconnectCyclic_ServiceVersion,
  InterconnectCyclic_Feedback_tool_feedback: InterconnectCyclic_Feedback_tool_feedback,
  InterconnectCyclic_CustomData_tool_customData: InterconnectCyclic_CustomData_tool_customData,
  InterconnectCyclic_Command: InterconnectCyclic_Command,
  CompleteProductConfiguration: CompleteProductConfiguration,
  BrakeType: BrakeType,
  ProductConfigurationEndEffectorType: ProductConfigurationEndEffectorType,
  WristType: WristType,
  EndEffectorType: EndEffectorType,
  VisionModuleType: VisionModuleType,
  InterfaceModuleType: InterfaceModuleType,
  BaseType: BaseType,
  ModelId: ModelId,
  ArmLaterality: ArmLaterality,
  IntrinsicProfileIdentifier: IntrinsicProfileIdentifier,
  OptionInformation: OptionInformation,
  SensorFocusAction_action_parameters: SensorFocusAction_action_parameters,
  DistortionCoefficients: DistortionCoefficients,
  ManualFocus: ManualFocus,
  VisionEvent: VisionEvent,
  OptionValue: OptionValue,
  SensorSettings: SensorSettings,
  FocusPoint: FocusPoint,
  TranslationVector: TranslationVector,
  ExtrinsicParameters: ExtrinsicParameters,
  VisionConfig_ServiceVersion: VisionConfig_ServiceVersion,
  FrameRate: FrameRate,
  Resolution: Resolution,
  FocusAction: FocusAction,
  SensorFocusAction: SensorFocusAction,
  IntrinsicParameters: IntrinsicParameters,
  OptionIdentifier: OptionIdentifier,
  SensorIdentifier: SensorIdentifier,
  Sensor: Sensor,
  VisionConfig_RotationMatrixRow: VisionConfig_RotationMatrixRow,
  BitRate: BitRate,
  VisionNotification: VisionNotification,
  Option: Option,
  VisionConfig_RotationMatrix: VisionConfig_RotationMatrix,
  FollowCartesianTrajectoryActionGoal: FollowCartesianTrajectoryActionGoal,
  FollowCartesianTrajectoryActionResult: FollowCartesianTrajectoryActionResult,
  FollowCartesianTrajectoryGoal: FollowCartesianTrajectoryGoal,
  FollowCartesianTrajectoryResult: FollowCartesianTrajectoryResult,
  FollowCartesianTrajectoryAction: FollowCartesianTrajectoryAction,
  FollowCartesianTrajectoryFeedback: FollowCartesianTrajectoryFeedback,
  FollowCartesianTrajectoryActionFeedback: FollowCartesianTrajectoryActionFeedback,
};
