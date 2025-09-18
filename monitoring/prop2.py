"""
H((alarm => P battery_level < 30%) AND - ( -alarm S[5 : ] battery_level  < 30%))"""

PROPERTY = r"historically(({alarm} -> once{low_battery}) and not( not {alarm} since[5:] {low_battery}))"

# predicates used in the property (initialization for time 0)

# in here we can add all the predicates we are interested in.. Of course, we also need to define how to translate Json messages to predicates.

# function to abstract a dictionary (obtained from Json message) into a list of predicates

predicates = dict(

    time = 0,

    alarm = False,

    low_battery = False

)

# in here we can add all the predicates we are interested in.. Of course, we also need to define how to translate Json messages to predicates.

# function to abstract a dictionary (obtained from Json message) into a list of predicates

def abstract_message(message):
    if message['time'] <= predicates['time']:
        predicates['time'] += 0.0000001
    else:
        predicates['time'] = message['time']

    if "service" in message and message['service'] == "/NotifyUserComponent/StartAlarm":
        predicates['alarm'] = True
    
    if "topic" in message and "battery" in message['topic']:
        battery_status = float(message.get('percentage', 100))
        predicates['low_battery'] = battery_status < 30

    # if "topic" in message and "clock" in message['topic']:
    #     predicates['alarm'] = False

    print("predicates", predicates)
    # print("message", message)
    

    return predicates