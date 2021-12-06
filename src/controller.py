import rospy
from std_msgs.msg import String
from demo_pakage.msg import Num


# CHANGE LANE - DEFAULT VALUE OF OPTINAL VALUE
DEFAULT_CHANGE_LANE_ALPHA = 30
DEFAULT_CHANGE_LANE_V_ANGULAR = 0.03
DEFAULT_CHANGE_LANE_DELTA_D = 0.5
DEFAULT_CHANGE_LANE_NO_USE = -1

# DETECT TRAFFIC SIGNAL DEFAULT VALUE
DEFAULT_DETECT_SIGNAL_LIVE_TIME = 15

# INVALID COMMAND
IVL_CMD = "Invalid command"

# LIST OF HEADER COMMAND
HEADER_CHANGE_LANE_STR = "change_lane"
HEADER_DETECT_SIGNAL_STR = "detect_signal"

def is_float(element):
    try:
        float(element)
        return True
    except ValueError:
        return False

def changeLaneFunc(params):    
    
    response_header = 1
    response_base_arg = DEFAULT_CHANGE_LANE_NO_USE
    response_alpha = DEFAULT_CHANGE_LANE_ALPHA
    response_v_ang = DEFAULT_CHANGE_LANE_V_ANGULAR
    response_delta_d = DEFAULT_CHANGE_LANE_DELTA_D     
    
    if(len(params) == 4 and params[0].isdigit() and params[1].isdigit() and is_float(params[2]) and is_float(params[3])):
        response_base_arg = int(params[0])
        response_alpha = int(params[1])
        response_v_ang = float(params[2])
        response_delta_d = float(params[3])
        return [response_header, response_base_arg, response_alpha, response_v_ang, response_delta_d] 
    
    # VERIFY IS EXIST BASE_ARG (MUST HAVE)
    if(params[0].isdigit()):
        response_base_arg = int(params[0])        
    elif(params[0].find("base_arg=") > -1):
        splitBaseArg = params[0].split("=")
        if(splitBaseArg[1].isdigit()):
            response_base_arg = int(splitBaseArg[1])                       
        else:
            return []
    else:
        return []        
    
    # VERIFY IS EIXST ALPHA
    for sub_param in params:
        if(sub_param.find("alpha=") > -1):
            splitAlpha = sub_param.split("=")
            if(splitAlpha[1].isdigit()):
                response_alpha = int(splitAlpha[1])                                   
            else:
                return []
    
    # VERIFY IS EXIST V_ANGULAR    
    for sub_param in params:
        if(sub_param.find("v_angular=") > -1):
            splitV = sub_param.split("=")
            if(is_float(splitV[1])):
                response_v_ang = float(splitV[1])                                
            else:
                return []
                    
    # VERIFY IS EXIST DELTA_D         
    for sub_param in params:
        if(sub_param.find("delta_d=") > -1):
            split_delta = sub_param.split("=")
            if(is_float(split_delta[1])):
                response_delta_d = float(split_delta[1])                                
            else:
                return []
        
    return [response_header, response_base_arg, response_alpha, response_v_ang, response_delta_d]   

pub = rospy.Publisher('controller_topic', Num, queue_size=10)
rospy.init_node('Controller', anonymous=True)

while not rospy.is_shutdown():
    cmd = input("~ ")
    splitCmd = cmd.split()        
    
    # Header of command
    if(len(splitCmd) > 1):
        header = splitCmd[0]
        if(header == HEADER_CHANGE_LANE_STR):
            returnData = changeLaneFunc(splitCmd[1: len(splitCmd)])
                        
        else:
            print(IVL_CMD)
    else:
        print(IVL_CMD)
        continue;  

    if(returnData != []):
        msg = Num()
        msg.header = returnData[0]
        msg.base_arg = returnData[1]
        msg.int_1 = returnData[2]
        msg.int_2 = DEFAULT_CHANGE_LANE_NO_USE
        msg.int_3 = DEFAULT_CHANGE_LANE_NO_USE
        msg.float_1 = returnData[3]
        msg.float_2 = returnData[4]
        msg.float_3 = DEFAULT_CHANGE_LANE_NO_USE
        pub.publish(msg)
    rate = rospy.Rate(10) # 10hz
    rate.sleep()

