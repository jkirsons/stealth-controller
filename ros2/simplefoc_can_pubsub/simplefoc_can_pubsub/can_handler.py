import can

class CanHandler:
    bus_node_ids = {}   # Registry of allocated IDs

    bitMask = [
        0b1111,         # DataType
        0b11111111,     # Command
        0b1111,         # MotorID
        0b111111111111  # BusID
        ]
    
    bitShift = [ 24, 16, 12, 0 ] # Bits to shift right/left by
    
    data_types = { "get value": 0,"float": 1,"double": 2,"unsigned char": 3,"int": 4,"long long": 5,"string": 6,"boolean": 7,"Bus Id": 8 }
    commands = { "SET_POS_ANG": 0, "CMD_STATUS": 1, "CMD_LIMITS|SCMD_LIM_CURR": 2, "CMD_LIMITS|SCMD_LIM_VOLT": 3, "CMD_LIMITS|SCMD_LIM_VEL": 4, "CMD_MOTION_TYPE": 5, "CMD_MOTION_TYPE|SCMD_DOWNSAMPLE": 6, "CMD_TORQUE_TYPE": 7, "CMD_SENSOR": 8, "CMD_SENSOR|SCMD_SENS_MECH_OFFSET": 9, "CMD_SENSOR|SCMD_SENS_ELEC_OFFSET": 10, "CMD_MONITOR": 11, "CMD_MONITOR|SCMD_DOWNSAMPLE": 12, "CMD_MONITOR|SCMD_CLEAR": 13, "CMD_MONITOR|SCMD_GET": 14, "CMD_MONITOR|SCMD_SET": 15, "CMD_RESIST": 16, "CMD_C_D_PID|SCMD_PID_P": 17, "CMD_C_D_PID|SCMD_PID_I": 18, "CMD_C_D_PID|SCMD_PID_D": 19, "CMD_C_D_PID|SCMD_PID_RAMP": 20, "CMD_C_D_PID|SCMD_PID_LIM": 21, "CMD_C_D_PID|SCMD_LPF_TF": 22, "CMD_C_Q_PID|SCMD_PID_P": 23, "CMD_C_Q_PID|SCMD_PID_I": 24, "CMD_C_Q_PID|SCMD_PID_D": 25, "CMD_C_Q_PID|SCMD_PID_RAMP": 26, "CMD_C_Q_PID|SCMD_PID_LIM": 27, "CMD_C_Q_PID|SCMD_LPF_TF": 28, "CMD_V_PID|SCMD_PID_P": 29, "CMD_V_PID|SCMD_PID_I": 30, "CMD_V_PID|SCMD_PID_D": 31, "CMD_V_PID|SCMD_PID_RAMP": 32, "CMD_V_PID|SCMD_PID_LIM": 33, "CMD_V_PID|SCMD_LPF_TF": 34, "CMD_A_PID|SCMD_PID_P": 35, "CMD_A_PID|SCMD_PID_I": 36, "CMD_A_PID|SCMD_PID_D": 37, "CMD_A_PID|SCMD_PID_RAMP": 38, "CMD_A_PID|SCMD_PID_LIM": 39, "CMD_A_PID|SCMD_LPF_TF": 40, "CMD_CUST_ALIGN": 41, "CMD_CUST_SAVE": 42, "CMD_CUST_LOAD": 43, "CMD_SCAN": 240, "CMD_CUST_REQ_BUS": 241, "CMD_CUST_BUS_ID": 242, "CMD_CUST_REBOOT": 243 }
    commands_reverse = { "": 0, "E": 1, "LC": 2, "LU": 3, "LV": 4, "C": 5, "CD": 6, "T": 7, "S": 8, "SM": 9, "SE": 10, "M": 11, "MD": 12, "MC": 13, "MG": 14, "MS": 15, "R": 16, "DP": 17, "DI": 18, "DD": 19, "DR": 20, "DL": 21, "DF": 22, "QP": 23, "QI": 24, "QD": 25, "QR": 26, "QL": 27, "QF": 28, "VP": 29, "VI": 30, "VD": 31, "VR": 32, "VL": 33, "VF": 34, "AP": 35, "AI": 36, "AD": 37, "AR": 38, "AL": 39, "AF": 40, "ZA": 41, "ZS": 42, "ZL": 43, "?": 240, "ZM": 241, "ZB": 242, "ZR": 243 }
    
    def __init__(self, bus):
        self.can_bus = bus

    def getBits(self, value, index):
        return (value >> CanHandler.bitShift[index]) & CanHandler.bitMask[index]

    def unpackId(self, value):
        return (self.getBits(value, 0), self.getBits(value, 1), self.getBits(value, 2), self.getBits(value, 3))

    def process(self, msg):
        (dataType, command, motorID, busID) = self.unpackId(msg.arbitration_id)
        
        # Requests for Bus ID
        if busID == 0:
            if bytes(msg.data) in self.bus_node_ids:
                self.send_can_message(self.data_types["Bus Id"], self.commands["CMD_CUST_BUS_ID"], 
                    0, self.bus_node_ids[bytes(msg.data)], msg.data)
            else:
                taken_nodes = self.bus_node_ids.values()
                for i in range(1, 4096):
                    if i not in taken_nodes:
                        self.bus_node_ids[bytes(msg.data)] = i
                        print("Allocating Bus ID: ", i, " to: ",bytes(msg.data).hex())
                        # send return message
                        self.send_can_message(self.data_types["Bus Id"], self.commands["CMD_CUST_BUS_ID"], 0, i, msg.data)
                        return
                print("No remaining Bus IDs to allocate")

    def send_can_message(self, data_type, command, motor_id, bus_node_id, data):
        id =  data_type << self.bitShift[0] | command << self.bitShift[1] | motor_id << self.bitShift[2] | bus_node_id << self.bitShift[3]
        msg = can.Message(arbitration_id=id, data=data, is_extended_id=True)
        try:
           self.can_bus.send(msg)
        except can.CanError:
            print("CAN Message NOT sent")
        return id
