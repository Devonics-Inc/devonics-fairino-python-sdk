"""
    @brief  Write Robot EtherCAT Slave File
    @param  [in] type Slave file type, 1 - upgrade slave file; 2 - upgrade slave configuration file
    @param  [in] slaveID Slave ID number
    @param  [in] fileName File name to be uploaded
    @return Error code: Success - 0, Failure - error code     
"""

@log_call
@xmlrpc_timeout
def SlaveFileWrite(self, type, slaveID, fileName):
    type = int(type)
    slaveID = int(slaveID)
    fileName = str(fileName)
    error = self.robot.SlaveFileWrite(type, slaveID, fileName)
    return error

"""
    @brief  Robot EtherCAT Slave Enter Boot Mode
    @return Error code: Success - 0, Failure - error code     
"""

@log_call
@xmlrpc_timeout
def SetSysServoBootMode(self):
    error = self.robot.SetSysServoBootMode()
    return error

"""
    @brief  Upload End-of-Arm Lua Open Protocol File
    @param  [in] filePath Local Lua file path name ".../AXLE_LUA_End_DaHuan.lua"
    @return Error code: Success - 0, Failure - error code     
"""

@log_call
@xmlrpc_timeout
def AxleLuaUpload(self, filePath):
    error = self.__FileUpLoad(10, filePath)
    file_name = "/tmp/" + os.path.basename(filePath)
    # file_name = os.path.basename(filePath)
    if error != 0:
        return error
    else:
        rtn = self.SetAxleFileType(2)
        if rtn != 0:
            return -1
        rtn = self.SetSysServoBootMode()
        if rtn != 0:
            return -1
        rtn = self.SlaveFileWrite(1, 7, file_name)
        if rtn != 0:
            return -1
        return rtn

"""
    *************************************************************************** New Additions ********************************************************************************************
"""

"""
    @brief  Enable Mobile Device
    @param  [in] enable Enable status, 0 - disable, 1 - enable
    @return Error code: Success - 0, Failure - error code     
"""

@log_call
@xmlrpc_timeout
def TractorEnable(self, enable):
    enable = int(enable)
    error = self.robot.TractorEnable(enable)
    return error

"""
    @brief  Mobile Device Homing
    @return Error code: Success - 0, Failure - error code     
"""

@log_call
@xmlrpc_timeout
def TractorHoming(self):
    error = self.robot.TractorHoming()
    return error

"""
    @brief  Mobile Device Linear Motion
    @param  [in] distance Linear motion distance (mm)
    @param  [in] vel Linear motion speed percentage (0-100)
    @return Error code: Success - 0, Failure - error code  
"""

@log_call
@xmlrpc_timeout
def TractorMoveL(self, distance, vel):
    if self.GetSafetyCode() != 0:
        return self.GetSafetyCode()
    distance = float(distance)
    vel = float(vel)
    error = self.robot.TractorMoveL(distance, vel)
    return error

"""
    @brief  Mobile Device Arc Motion
    @param  [in] radio Arc motion radius (mm)
    @param  [in] angle Arc motion angle (°)
    @param  [in] vel Arc motion speed percentage (0-100)
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def TractorMoveC(self, radio, angle, vel):
    if self.GetSafetyCode() != 0:
        return self.GetSafetyCode()
    radio = float(radio)
    angle = float(angle)
    vel = float(vel)
    error = self.robot.TractorMoveC(radio, angle, vel)
    return error

"""
    @brief  Mobile Device Stop Motion
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def TractorStop(self):
    error = self.robot.ProgramStop()
    return error

"""
    @brief  Set Welding Wire Search Extended IO Ports
    @param  [in] searchDoneDINum DO port for successful welding wire search (0-127)
    @param  [in] searchStartDONum DO port for welding wire search start/stop control (0-127)
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def SetWireSearchExtDIONum(self, searchDoneDINum, searchStartDONum):
    searchDoneDINum = int(searchDoneDINum)
    searchStartDONum = int(searchStartDONum)
    error = self.robot.SetWireSearchExtDIONum(searchDoneDINum, searchStartDONum)
    return error

"""
    @brief  Set Welding Machine Control Mode Extended DO Port
    @param  [in] DONum DO port for welding machine control mode (0-127)
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def SetWeldMachineCtrlModeExtDoNum(self, DONum):
    DONum = int(DONum)
    error = self.robot.SetWeldMachineCtrlModeExtDoNum(DONum)
    return error

"""
    @brief  Set Welding Machine Control Mode
    @param  [in] mode Welding machine control mode; 0 - unified mode
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def SetWeldMachineCtrlMode(self, mode):
    mode = int(mode)
    error = self.robot.SetWeldMachineCtrlMode(mode)
    return error

"""
    @brief  Close RPC
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def CloseRPC(self):
    # Set stop event to signal thread to stop
    self.stop_event.set()

    # If thread is still running, wait for it to finish
    # if self.thread.is_alive():
    #     self.thread.join()

    # Clean up XML-RPC proxy
    if self.robot is not None:
        self.robot = None  # Set proxy to None to release resources
        self.sock_cli_state.close()
        self.sock_cli_state = None
        self.robot_state_pkg = None
        self.closeRPC_state = True
        # self.robot_realstate_exit = False

    # If thread is still running, wait for it to finish
    if self.thread.is_alive():
        self.thread.join()

    print("RPC connection closed.")
    return

"""
    @brief  Record Teaching Point
    @param  [in] name Name of the teaching point
    @param  [in] update_allprogramfile Whether to overwrite, 0 - do not overwrite, 1 - overwrite
    @return Error code: Success - 0, Failure - error code
"""

# @log_call
# @xmlrpc_timeout
#
# def SavePoint(self, name, update_allprogramfile=0):
#     name = str(name)
#     update_allprogramfile = int(update_allprogramfile)
#     error = self.robot.save_point(name, update_allprogramfile)
#     return error

"""
    @brief  Start Singular Pose Protection
    @param  [in] protectMode Singular protection mode, 0: Joint mode; 1 - Cartesian mode
    @param  [in] minShoulderPos Shoulder singular adjustment range (mm), default 100.0
    @param  [in] minElbowPos Elbow singular adjustment range (mm), default 50.0
    @param  [in] minWristPos Wrist singular adjustment range (°), default 10.0
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def SingularAvoidStart(self, protectMode, minShoulderPos=100, minElbowPos=50, minWristPos=10):
    protectMode = int(protectMode)
    minShoulderPos = float(minShoulderPos)
    minElbowPos = float(minElbowPos)
    minWristPos = float(minWristPos)
    error = self.robot.SingularAvoidStart(protectMode, minShoulderPos, minElbowPos, minWristPos)
    return error

"""
    @brief  Stop Singular Pose Protection
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def SingularAvoidEnd(self):
    error = self.robot.SingularAvoidEnd()
    return error

"""
    @brief  Get Rotational Gripper Rotation Count
    @return Error code: Success - 0, Failure - error code
    @return Return value (on success) fault: 0 - no error, 1 - error present
    @return Return value (on success) num: Rotation count
"""

@log_call
@xmlrpc_timeout
def GetGripperRotNum(self):
    return 0, self.robot_state_pkg.gripper_fault, self.robot_state_pkg.gripperRotNum

"""
    @brief  Get Rotational Gripper Rotation Speed Percentage
    @return Error code: Success - 0, Failure - error code
    @return Return value (on success) fault: 0 - no error, 1 - error present
    @return Return value (on success) speed: Rotation speed percentage
"""

@log_call
@xmlrpc_timeout
def GetGripperRotSpeed(self):
    return 0, self.robot_state_pkg.gripper_fault, self.robot_state_pkg.gripperRotSpeed

"""
    @brief  Get Rotational Gripper Rotation Torque Percentage
    @return Error code: Success - 0, Failure - error code
    @return Return value (on success) fault: 0 - no error, 1 - error present
    @return Return value (on success) torque: Rotation torque percentage
"""

@log_call
@xmlrpc_timeout
def GetGripperRotTorque(self):
    return 0, self.robot_state_pkg.gripper_fault, self.robot_state_pkg.gripperRotTorque

"""
    @brief  Start PTP Motion FIR Filtering
    @param  [in] maxAcc Maximum acceleration limit (deg/s^2)
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def PtpFIRPlanningStart(self, maxAcc):
    maxAcc = float(maxAcc)
    error = self.robot.PtpFIRPlanningStart(maxAcc)
    return error

"""
    @brief  End PTP Motion FIR Filtering
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def PtpFIRPlanningEnd(self):
    error = self.robot.PtpFIRPlanningEnd()
    return error

"""
    @brief  Upload Trajectory J File
    @param  [in] filePath Full path name of the trajectory file to be uploaded   C://test/testJ.txt
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def TrajectoryJUpLoad(self, filePath):
    error = self.__FileUpLoad(20, filePath)
    return error

"""2024.12.16"""
"""
    @brief  Delete Trajectory J File
    @param  [in] fileName Full path name of the trajectory file to be deleted   C://test/testJ.txt
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def TrajectoryJDelete(self, fileName):
    error = self.__FileDelete(20, fileName)
    return error

"""2024.12.18"""
"""
    @brief  Start LIN/ARC Motion FIR Filtering
    @param  [in] maxAccLin Linear acceleration limit (mm/s^2)
    @param  [in] maxAccDeg Angular acceleration limit (deg/s^2)
    @param  [in] maxJerkLin Linear jerk limit (mm/s^3)
    @param  [in] maxJerkDeg Angular jerk limit (deg/s^3)
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def LinArcFIRPlanningStart(self, maxAccLin, maxAccDeg, maxJerkLin, maxJerkDeg):
    maxAccLin = float(maxAccLin)
    maxAccDeg = float(maxAccDeg)
    maxJerkLin = float(maxJerkLin)
    maxJerkDeg = float(maxJerkDeg)
    error = self.robot.LinArcFIRPlanningStart(maxAccLin, maxAccDeg, maxJerkLin, maxJerkDeg)
    return error

"""
    @brief  End LIN/ARC Motion FIR Filtering
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def LinArcFIRPlanningEnd(self):
    error = self.robot.LinArcFIRPlanningEnd()
    return error

"""2025.01.08"""
"""
    @brief  Start Tool Coordinate System Transformation
    @param  [in] toolNum Tool coordinate system number [0-14]
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def ToolTrsfStart(self, toolNum):
    toolNum = int(toolNum)
    error = self.robot.ToolTrsfStart(toolNum)
    return error

"""
    @brief  End Tool Coordinate System Transformation
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def ToolTrsfEnd(self):
    error = self.robot.ToolTrsfEnd()
    return error

"""2025.01.08"""
"""3.7.8"""
"""
    @brief  Calculate Tool Coordinate System Based on Point Information
    @param  [in] method Calculation method; 0 - four-point method; 1 - six-point method
    @param  [in] pos Joint position group, length of array is 4 for four-point method and 6 for six-point method
    @return Error code: Success - 0, Failure - error code
    @return Return value (on success) tcp_offset = [x, y, z, rx, ry, rz]: Tool coordinate system calculated from point information, units [mm] [°]
"""

@log_call
@xmlrpc_timeout
def ComputeToolCoordWithPoints(self, method, pos):
    method = int(method)
    param = {}
    param[0] = pos[0]
    param[1] = pos[1]
    param[2] = pos[2]
    param[3] = pos[3]

    if method == 0:  # Four-point method
        param[4] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        param[5] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    else:  # Six-point method
        param[4] = pos[4]
        param[5] = pos[5]
    _error = self.robot.ComputeToolCoordWithPoints(method, param[0], param[1], param[2], param[3], param[4], param[5])
    error = _error[0]
    if error == 0:
        return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
    return error

"""
    @brief  Calculate Workpiece Coordinate System Based on Point Information
    @param  [in] method Calculation method; 0: origin-x axis-z axis  1: origin-x axis-xy plane
    @param  [in] pos Three TCP position groups
    @param  [in] refFrame Reference coordinate system
    @return Error code: Success - 0, Failure - error code
    @return Return value (on success) wobj_offset = [x, y, z, rx, ry, rz]: Workpiece coordinate system calculated from point information, units [mm] [°]
"""

@log_call
@xmlrpc_timeout
def ComputeWObjCoordWithPoints(self, method, pos, refFrame):
    method = int(method)
    param = {}
    param[0] = pos[0]
    param[1] = pos[1]
    param[2] = pos[2]
    refFrame = int(refFrame)
    _error = self.robot.ComputeWObjCoordWithPoints(method, param[0], param[1], param[2], refFrame)
    error = _error[0]
    if error == 0:
        return error, [_error[1], _error[2], _error[3], _error[4], _error[5], _error[6]]
    return error

"""
    @brief  Set Robot Welding Arc Interruption Detection Parameters
    @param  [in] checkEnable Whether to enable detection; 0 - disable; 1 - enable
    @param  [in] arcInterruptTimeLength Duration to confirm arc interruption (ms)
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def WeldingSetCheckArcInterruptionParam(self, checkEnable, arcInterruptTimeLength):
    checkEnable = int(checkEnable)
    arcInterruptTimeLength = int(arcInterruptTimeLength)
    error = self.robot.WeldingSetCheckArcInterruptionParam(checkEnable, arcInterruptTimeLength)
    return error

"""
    @brief  Get Robot Welding Arc Interruption Detection Parameters
    @return Error code: Success - 0, Failure - error code
    @return Return value (on success) checkEnable: Whether detection is enabled; 0 - disable; 1 - enable
    @return Return value (on success) arcInterruptTimeLength: Duration to confirm arc interruption (ms)
"""

@log_call
@xmlrpc_timeout
def WeldingGetCheckArcInterruptionParam(self):
    _error = self.robot.WeldingGetCheckArcInterruptionParam()
    error = _error[0]
    if error == 0:
        return error, _error[1], _error[2]
    return error

"""
    @brief  Set Robot Welding Interrupt Recovery Parameters
    @param  [in] enable Whether to enable welding interrupt recovery
    @param  [in] length Overlap distance of welding seam (mm)
    @param  [in] velocity Speed percentage (0-100) for the robot to return to the re-arc point
    @param  [in] moveType Robot motion method to reach the re-arc point; 0 - LIN; 1 - PTP
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def WeldingSetReWeldAfterBreakOffParam(self, enable, length, velocity, moveType):
    enable = int(enable)
    length = float(length)
    velocity = float(velocity)
    moveType = int(moveType)
    error = self.robot.WeldingSetReWeldAfterBreakOffParam(enable, length, velocity, moveType)
    return error

"""
    @brief  Get Robot Welding Interrupt Recovery Parameters
    @return Error code: Success - 0, Failure - error code
    @return Return value (on success) enable: Whether welding interrupt recovery is enabled
    @return Return value (on success) length: Overlap distance of welding seam (mm)
    @return Return value (on success) velocity: Speed percentage (0-100) for the robot to return to the re-arc point
    @return Return value (on success) moveType: Robot motion method to reach the re-arc point; 0 - LIN; 1 - PTP
"""

@log_call
@xmlrpc_timeout
def WeldingGetReWeldAfterBreakOffParam(self):
    _error = self.robot.WeldingGetReWeldAfterBreakOffParam()
    error = _error[0]
    if error == 0:
        return error, _error[1], _error[2], _error[3], _error[4]
    return error

"""
    @brief  Set Robot to Resume Welding After Interrupt
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def WeldingStartReWeldAfterBreakOff(self):
    error = self.robot.WeldingStartReWeldAfterBreakOff()
    return error

"""
    @brief  Set Robot to Abort Welding After Interrupt
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def WeldingAbortWeldAfterBreakOff(self):
    error = self.robot.WeldingAbortWeldAfterBreakOff()
    return error

"""2025.01.09"""
"""
    @brief 
    @param  [in] status
    @param  [in] delayMode
    @param  [in] delayTime
    @param  [in] delayDisExAxisNum
    @param  [in] delayDis
    @param  [in] sensitivePara
    @param  [in] speed
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def LaserSensorRecord(self, status, delayMode, delayTime, delayDisExAxisNum, delayDis, sensitivePara, speed):
    status = int(status)
    delayMode = int(delayMode)
    delayTime = int(delayTime)
    delayDisExAxisNum = int(delayDisExAxisNum)
    delayDis = float(delayDis)
    sensitivePara = float(sensitivePara)
    speed = float(speed)
    error = self.robot.LaserSensorRecord(status, delayMode, delayTime, delayDisExAxisNum, delayDis, sensitivePara, speed)
    return error

"""
    @brief 
    @param  [in] weldId
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def LaserTrackingLaserOn(self, weldId):
    weldId = int(weldId)
    error = self.robot.LaserTrackingLaserOn(weldId)
    return error

"""
    @brief 
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def LaserTrackingLaserOff(self):
    error = self.robot.LaserTrackingLaserOff()
    return error

"""
    @brief 
    @param  [in] coordId
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def LaserTrackingTrackOn(self, coordId):
    coordId = int(coordId)
    error = self.robot.LaserTrackingTrackOn(coordId)
    return error

"""
    @brief 
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def LaserTrackingTrackOff(self):
    error = self.robot.LaserTrackingTrackOff()
    return error

"""
    @brief 
    @param  [in] direction
    @param  [in] directionPoint
    @param  [in] vel
    @param  [in] distance
    @param  [in] timeout
    @param  [in] posSensorNum
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def LaserTrackingSearchStart(self, direction, directionPoint, vel, distance, timeout, posSensorNum):
    direction = int(direction)
    directionPoint = list(map(float, directionPoint))
    vel = int(vel)
    distance = int(distance)
    timeout = int(timeout)
    posSensorNum = int(posSensorNum)
    error = self.robot.LaserTrackingSearchStart(direction, directionPoint, vel, distance, timeout, posSensorNum)
    return error

"""
    @brief 
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def LaserTrackingSearchStop(self):
    error = self.robot.LaserTrackingSearchStop()
    return error

"""2025.01.24"""
"""3.7.9"""
"""
    @brief  Start Weave Gradation
    @param  [in] weaveNum Weave number
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def WeaveChangeStart(self, weaveNum):
    weaveNum = int(weaveNum)
    error = self.robot.WeaveChangeStart(weaveNum)
    return error

"""
    @brief  End Weave Gradation
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def WeaveChangeEnd(self):
    error = self.robot.WeaveChangeEnd()
    return error

"""2025.02.20"""
"""3.8.0"""
"""
    @brief  Trajectory Preprocessing (Trajectory Look-Ahead)
    @param  [in] name  Trajectory file name
    @param  [in] mode Sampling mode, 0 - no sampling; 1 - equal data interval sampling; 2 - equal error limit sampling
    @param  [in] errorLim Error limit, effective when using straight line fitting
    @param  [in] type Smoothing method, 0 - Bezier smoothing
    @param  [in] precision Smoothing precision, effective when using Bezier smoothing
    @param  [in] vamx Set maximum speed, mm/s
    @param  [in] amax Set maximum acceleration, mm/s^2
    @param  [in] jmax Set maximum jerk, mm/s^3
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def LoadTrajectoryLA(self, name, mode, errorLim, type, precision, vamx, amax, jmax):
    name = str(name)
    mode = int(mode)
    errorLim = float(errorLim)
    type = int(type)
    precision = float(precision)
    vamx = float(vamx)
    amax = float(amax)
    jmax = float(jmax)
    error = self.robot.LoadTrajectoryLA(name, mode, errorLim, type, precision, vamx, amax, jmax)
    return error

"""
    @brief  Trajectory Playback (Trajectory Look-Ahead)
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def MoveTrajectoryLA(self):
    error = self.robot.MoveTrajectoryLA()
    return error

"""2025.02.25"""
"""
    @brief  Custom Collision Detection Threshold Function Start, Set Collision Detection Threshold for Joint and TCP
    @param  [in] flag 1 - Only joint detection enabled; 2 - Only TCP detection enabled; 3 - Both joint and TCP detection enabled
    @param  [in] jointDetectionThreshould Joint collision detection threshold for j1 - j6
    @param  [in] tcpDetectionThreshould TCP collision detection threshold for xyzabc
    @param  [in] block 0 - Non-blocking; 1 - Blocking
    @return  Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def CustomCollisionDetectionStart(self, flag, jointDetectionThreshould, tcpDetectionThreshould, block):
    flag = int(flag)
    jointDetectionThreshould = list(map(float, jointDetectionThreshould))
    tcpDetectionThreshould = list(map(float, tcpDetectionThreshould))
    block = int(block)
    error = self.robot.CustomCollisionDetectionStart(flag, jointDetectionThreshould, tcpDetectionThreshould, block)
    return error

"""
    @brief  Custom Collision Detection Threshold Function End
    @return Error code: Success - 0, Failure - error code
"""

@log_call
@xmlrpc_timeout
def CustomCollisionDetectionEnd(self):
    error = self.robot.CustomCollisionDetectionEnd()
    return error
