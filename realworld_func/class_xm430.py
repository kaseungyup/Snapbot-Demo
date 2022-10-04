import os, ctypes, time
import realworld_func.custom_dynamixel_functions as dynamixel
import numpy as np

class xm430(object):
    """
        CONSTRUCTOR
    """
    def __init__(self, name, USB_NUM, VERBOSE=True):
        self.name    = name
        self.USB_NUM = USB_NUM
        self.VERBOSE = VERBOSE
        self.define_addr()
        if self.VERBOSE:
            print ("[%s] INSTANTIATED AT [%s]" % (self.name, self.DEVICENAME))

    """
        DEFAULT INITAILIZER FOR A SNAPBOT WITH FOUR LEGS WITH TWO JOINTS
    """
    def get_fourleged_snapbot(self):
        self.connect()
        min_pos = np.asarray([274, 272, 274, 282, 287, 275, 273, 267])
        max_pos = np.asarray([765, 671, 737, 666, 736, 671, 743, 668])
        self.set_minmaxpos(min_pos, max_pos), time.sleep(1e-3)
        self.set_delaytime([10]), time.sleep(1e-3)
        self.set_pidgains(50, 0, 0), time.sleep(1e-3)
        self.set_maxtorque([1000]), time.sleep(1e-3)
        self.set_goalspeed([700]), time.sleep(1e-3)
        self.set_torque([1]), time.sleep(1e-3)
        # self.set_goalpos([512]), time.sleep(1e-3)


        # self.IDX_LIST = [15, 16, 19, 20, 21, 22, 17, 18]
        # self.IDX_LIST = [15, 16, 19, 20, 21, 22, 17, 18]
        print (self.IDX_LIST)

    def get_fourleged_snapbot_slow(self):
        self.connect()
        min_pos = np.asarray([274, 272, 274, 282, 287, 275, 273, 267])
        max_pos = np.asarray([765, 671, 737, 666, 736, 671, 743, 668])
        self.set_minmaxpos(min_pos, max_pos), time.sleep(1e-3)
        self.set_delaytime([10]), time.sleep(1e-3)
        self.set_pidgains(32, 0, 0), time.sleep(1e-3)
        self.set_maxtorque([1000]), time.sleep(1e-3)
        self.set_goalspeed([600]), time.sleep(1e-3)
        self.set_torque([1]), time.sleep(1e-3)
        # self.set_goalpos([512]), time.sleep(1e-3)


        # self.IDX_LIST = [15, 16, 19, 20, 21, 22, 17, 18]
        # self.IDX_LIST = [15, 16, 19, 20, 21, 22, 17, 18]
        print (self.IDX_LIST)

    """
        DEFINE ADDRESSES FOR READ AND WRITE FOR XL-320
    """
    def define_addr(self):
        # BASIC ADDRESSES AND LENGTH OF COMMANDS
        self.ADDR_TORQUE_ENABLE       = 64
        self.LEN_TORQUE_ENABLE        = 1
        self.ADDR_PGAIN               = 29
        self.LEN_PGAIN                = 1
        self.ADDR_IGAIN               = 28
        self.LEN_IGAIN                = 1
        self.ADDR_DGAIN               = 27
        self.LEN_DGAIN                = 1
        self.ADDR_GOAL_POSITION       = 116
        self.LEN_GOAL_POSITION        = 4
        self.ADDR_GOAL_VELOCITY       = 104
        self.LEN_GOAL_VELOCITY        = 4
        self.ADDR_PRESENT_CURRENT    = 126
        self.LEN_PRESENT_CURRENT    = 2
        self.ADDR_PRESENT_POSITION    = 132
        self.LEN_PRESENT_POSITION     = 4
        self.ADDR_TORQUE_LIMIT        = 35
        self.LEN_TORQUE_LIMIT         = 2
        self.ADDR_DELAY_TIME          = 5
        self.LEN_DELAY_TIME           = 1
        self.ADDR_LED                 = 65
        self.LEN_LED                  = 1
        self.ADDR_OPERATING_MODE      = 11
        self.LEN_OPERATING_MODE       = 1


        self.ADDR_CW_ANGLE_LIMIT      = 6
        self.LEN_CW_ANGLE_LIMIT       = 2
        self.ADDR_CCW_ANGLE_LIMIT     = 8
        self.LEN_CCW_ANGLE_LIMIT      = 2

        # BASIC CONFIGURATION
        self.PROTOCOL_VERSION         = 2
        self.DXL_ID                   = 1
        self.BAUDRATE                 = 1000000 # 57600 / 1000000
        self.DEVICENAME               = ("/dev/ttyUSB%d"%(self.USB_NUM)).encode('utf-8')
        self.MAX_ID                   = 252
        self.COMM_SUCCESS             = 0
        self.COMM_TX_FAIL             = -1001

    """
        CONNECT TO XL-320
    """
    def connect(self, _EXPECTED_NID=0):
        if self.VERBOSE:
            print ("[%s] CONNECT" % (self.name))
        # dynamixel.closePort(0)
        self.port_num = dynamixel.portHandler(self.DEVICENAME)
        dynamixel.packetHandler()
        dxl_comm_result = self.COMM_TX_FAIL
        # OPEN PORT
        if dynamixel.openPort(self.port_num):
            if self.VERBOSE:
                print(" SUCCEEDED TO OPEN THE PORT!")
        else:
            if self.VERBOSE:
                print(" FAILED TO OPEN THE PORT!")
        if dynamixel.setBaudRate(self.port_num, self.BAUDRATE):
            if self.VERBOSE:
                print(" SUCCEEDED TO CHANGE THE BAUDRATE!")
        else:
            if self.VERBOSE:
                print(" FAILED TO CHANGE THE BAUDRATE!")
        # CONNECT TO DYNAMIXEL
        dynamixel.broadcastPing(self.port_num, self.PROTOCOL_VERSION)
        dxl_comm_result = dynamixel.getLastTxRxResult(self.port_num, self.PROTOCOL_VERSION)
        if dxl_comm_result != self.COMM_SUCCESS:
            if self.VERBOSE:
                print( " COMM FAIL [%s]" %
                    (dynamixel.getTxRxResult(self.PROTOCOL_VERSION, dxl_comm_result)))
        else:
            if self.VERBOSE:
                print ("COMMUNICATION SUCCESS")
        # DETECTED MOTORS
        time.sleep(0.1)
        self.detect_idxs()
        if (_EXPECTED_NID>0) & (self.NID!=_EXPECTED_NID):
            if self.VERBOSE:
                print ("\n\n\n[%s] NID DOES NOT MATCH\n\n\n" % (self.name))
        # UPDATE CURRENT POSITION
        self.get_currpos()
        # LED THING
        for i in range(8):
            self.set_led([i])
            time.sleep(0.1)
        self.set_led([0])
    """
        CLOSE XL-320
    """
    def close(self):
        dynamixel.closePort(self.port_num)
        if self.VERBOSE:
            print ("[%s] CLOSE" % (self.name))

    """
        CHECK IDs => self.IDX_LIST
    """
    def detect_idxs(self):
        self.IDX_LIST = []
        if self.VERBOSE:
            print ("[%s] DETECT ID" % (self.name))
        for id in range(0, self.MAX_ID):
            time.sleep(0.001)
            if ctypes.c_ubyte(dynamixel.getBroadcastPingResult(self.port_num,
                    self.PROTOCOL_VERSION, id)).value:
                # IF PINGED
                self.IDX_LIST.append(id)
                # print("[ID:%03d]" % (id))
        self.NID = len(self.IDX_LIST)
        if self.VERBOSE:
            print ("[%s] [%d]MOTORS DETECTED " % (self.name, self.NID))
            print ("[%s] ID_LIST:[%s]" % (self.name, self.IDX_LIST))

    """
        SYNCREAD
    """
    def syncread(self, _ADDR, _LEN, _NTRY=5):
        # GET GROUP NUMBER
        _groupread_num = dynamixel.groupSyncRead(self.port_num,
            self.PROTOCOL_VERSION, _ADDR, _LEN)
        # ADD THINGS TO READ
        for _idx in self.IDX_LIST:
            _dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncReadAddParam(
                _groupread_num, _idx)).value
            if _dxl_addparam_result != 1:
                if self.VERBOSE:
                    print("[SYNCREAD][ID:%03d]groupSyncReadAddParam FAILED" % (_idx))
        # DO SYNC READ
        _read_count = 0
        _outval = [0]*len(self.IDX_LIST)
        while True:
            _read_count = _read_count + 1
            time.sleep(1e-3)
            dynamixel.groupSyncReadTxRxPacket(_groupread_num)

            if _NTRY==0:
                for _i, _idx in enumerate(self.IDX_LIST):
                    _outval[_i] = dynamixel.groupSyncReadGetData(_groupread_num,
                        _idx, _ADDR, _LEN)
                break

            _dxl_comm_result = dynamixel.getLastTxRxResult(self.port_num,
                self.PROTOCOL_VERSION)
            if _dxl_comm_result != self.COMM_SUCCESS:
                _is_success = 0
            else:
                _is_success = 1
            # IF SUCCESS
            if _is_success:
                for _i, _idx in enumerate(self.IDX_LIST):
                    _outval[_i] = dynamixel.groupSyncReadGetData(_groupread_num,
                        _idx, _ADDR, _LEN)
                break
            # TOO MANY TRIES..
            if _read_count > _NTRY:
                if self.VERBOSE:
                    print ("[%s] SYNCREAD FAIL" % (self.name))
                break

        # CLEAR PARAMETERS
        dynamixel.groupSyncReadClearParam(_groupread_num)
        return _outval

    """
        SYNCWRITE
    """
    def syncwrite(self, _ADDR, _LEN, _VAL_LIST, _NTRY=5):
        # GET GROUP NUMBER
        _groupwrite_num = dynamixel.groupSyncWrite(self.port_num,
            self.PROTOCOL_VERSION, _ADDR, _LEN)
        # DO SYNC WRITE
        _write_count = 0
        while True:
            _write_count = _write_count + 1
            # ADD THINGS TO WRITE
            _is_success = 1
            for _idx, _val in zip(self.IDX_LIST, _VAL_LIST):
                addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(_groupwrite_num, _idx, int(_val), _LEN)).value
                # if ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(
                #     _groupwrite_num, _idx, _val, _LEN)).value != 1:
                if addparam_result != 1:
                    _is_success = 0
                    if self.VERBOSE:
                        print("[SYNCWRITE][ID:%03d] FAILED" % (_idx))
            dynamixel.groupSyncWriteTxPacket(_groupwrite_num)

            if _NTRY==0:
                break

            time.sleep(1e-3)
            _dxl_comm_result = dynamixel.getLastTxRxResult(self.port_num,
                self.PROTOCOL_VERSION)
            if _dxl_comm_result != self.COMM_SUCCESS:
                _is_success = 0
            # IF SUCCESS, BREAK OUT
            if _is_success:
                break
            # TOO MANY TRIES..
            if _write_count > _NTRY:
                if self.VERBOSE:
                    print ("[%s] SYNCWRITE FAIL" % (self.name))
                break

        # CLEAR PARAMETERS
        dynamixel.groupSyncWriteClearParam(_groupwrite_num)


    """                                             """
    """   LITTLE HIGH LEVEL WRAPPERS                """
    """   INPUT CAN EITHER BE 'LIST' OR 'NDARRAY'   """
    """   OUTPUT IS 'NDARRAY'                       """
    """                                             """

    """
        GET CURRENT POSITION
    """
    def get_currpos(self, _NTRY=5):
        time.sleep(1e-3)
        curr_pos = np.asarray(self.syncread(self.ADDR_PRESENT_POSITION,
                    self.LEN_PRESENT_POSITION, _NTRY))
        self.curr_pos = curr_pos
        if curr_pos.min() == 0:
            if self.VERBOSE:
                print ("[GET_CURRPOS] ERROR, PROBABILLY [%s]" % (curr_pos))
        return curr_pos

    """
        GET CURRENT CURRENT
    """
    def get_currcurr(self, _NTRY=5):
        time.sleep(1e-3)
        curr_curr = np.asarray(self.syncread(self.ADDR_PRESENT_CURRENT,
                    self.LEN_PRESENT_CURRENT, _NTRY))
        self.curr_curr = curr_curr
        print(curr_curr)
        # if curr_curr.min() == 0:
        #     if self.VERBOSE:
        #         print ("[GET_CURRPOS] ERROR, PROBABILLY [%s]" % (curr_curr))
        return curr_curr

    """
        GET ANGLE LIMIT
    """
    def get_anglelimit(self, _NTRY=5):
        time.sleep(1e-3)
        cw_anglelimit = np.asarray(self.syncread(self.ADDR_CW_ANGLE_LIMIT,
                    self.LEN_CW_ANGLE_LIMIT, _NTRY))
        ccw_anglelimit = np.asarray(self.syncread(self.ADDR_CCW_ANGLE_LIMIT,
                    self.LEN_CCW_ANGLE_LIMIT, _NTRY))
        anglelimits = [cw_anglelimit,ccw_anglelimit]
        return anglelimits


    """
        SET WORKING RANGE OF MOTORS
    """
    def set_minmaxpos(self, min_pos, max_pos):
        self.min_pos = min_pos
        self.max_pos = max_pos

    """
        SET PID GAINS
    """
    def set_pidgains(self, PGAIN, IGAIN, DGAIN, _NTRY=5):
        self.syncwrite(self.ADDR_PGAIN,
                 self.LEN_PGAIN, [PGAIN]*self.NID, _NTRY)
        self.syncwrite(self.ADDR_IGAIN,
                 self.LEN_IGAIN, [IGAIN]*self.NID, _NTRY)
        self.syncwrite(self.ADDR_DGAIN,
                 self.LEN_DGAIN, [DGAIN]*self.NID, _NTRY)
    """
        SET DELAY TIME (0~254 / 250)
    """
    def set_delaytime(self, delay_time, _NTRY=5):
        if len(delay_time) == 1:
            delay_time = delay_time*self.NID
        self.syncwrite(self.ADDR_DELAY_TIME,
                 self.LEN_DELAY_TIME, delay_time, _NTRY)

    """
        SET TORQUE (ON/OFF)
    """
    def set_torque(self, ON_FLAG, _NTRY=5):
        if len(ON_FLAG) == 1:
            ON_FLAG = ON_FLAG*self.NID
        self.syncwrite(self.ADDR_TORQUE_ENABLE,
                 self.LEN_TORQUE_ENABLE, ON_FLAG, _NTRY)
    """
        SET GOAL POSITION (0~1023)
    """
    def set_goalpos(self, GOAL_POS, _NTRY=5):
        time.sleep(5e-3)
        # SET MIN MAX THRESHOLDING
        GOAL_POS = np.asarray(GOAL_POS)
        # GOAL_POS = np.maximum(GOAL_POS, self.min_pos)
        # GOAL_POS = np.minimum(GOAL_POS, self.max_pos)
        # GET GOAL POS
        self.goal_pos = GOAL_POS
        # THEN MOVE
        self.syncwrite(self.ADDR_GOAL_POSITION,
                 self.LEN_GOAL_POSITION, GOAL_POS, _NTRY)
    """
        GET GOAL POSITION (0~1023)
    """
    def get_goalpos(self, _NTRY=5):
        # THEN MOVE
        _goal_pos = self.syncread(self.ADDR_GOAL_POSITION,
                 self.LEN_GOAL_POSITION, _NTRY)
        return _goal_pos
    """
        SET GOAL SPEED (0~2047)
    """
    def set_goalspeed(self, GOAL_VEL, _NTRY=5):
        if len(GOAL_VEL) == 1:
            GOAL_VEL = GOAL_VEL*self.NID
        self.goal_vel = GOAL_VEL
        self.syncwrite(self.ADDR_GOAL_VELOCITY,
                 self.LEN_GOAL_VELOCITY, GOAL_VEL, _NTRY)
    """
        SET TORQUE SPEED (0~1023)
    """
    def set_maxtorque(self, MAX_TORQUE, _NTRY=5):
        if len(MAX_TORQUE) == 1:
            MAX_TORQUE = MAX_TORQUE*self.NID
        self.max_torque = MAX_TORQUE
        self.syncwrite(self.ADDR_TORQUE_LIMIT,
                 self.LEN_TORQUE_LIMIT, MAX_TORQUE, _NTRY)
    """
        SET LED (0~7)
    """
    def set_led(self, LED_VAL, _NTRY=5):
        if len(LED_VAL) == 1:
            LED_VAL = LED_VAL*self.NID
        self.LED_VAL = LED_VAL
        self.syncwrite(self.ADDR_LED,
                 self.LEN_LED, self.LED_VAL, _NTRY)

    """
        SET OPERATING MODE (0~16)
    """
    def set_operatingmode(self, OPERATING_MODE_VAL, _NTRY=5):
        if len(OPERATING_MODE_VAL) == 1:
            OPERATING_MODE_VAL = OPERATING_MODE_VAL*self.NID
        self.OPERATING_MODE_VAL = OPERATING_MODE_VAL
        self.syncwrite(self.ADDR_OPERATING_MODE,
                 self.LEN_OPERATING_MODE, self.OPERATING_MODE_VAL, _NTRY)



























# JUST FOR MARGIN
