"""
H(battery_level > 30% IMPLIES H(-alarm))"""

PROPERTY = r"historically({high_battery} -> historically( not {alarm}))"

# predicates used in the property (initialization for time 0)

# in here we can add all the predicates we are interested in.. Of course, we also need to define how to translate Json messages to predicates.

# function to abstract a dictionary (obtained from Json message) into a list of predicates

predicates = dict(

    time = 0,

    alarm = False,

    high_battery = True

)

# in here we can add all the predicates we are interested in.. Of course, we also need to define how to translate Json messages to predicates.

# function to abstract a dictionary (obtained from Json message) into a list of predicates

def abstract_message(message):
    if message['time'] <= predicates['time']:
        predicates['time'] += 0.0000001
    else:
        predicates['time'] = message['time']

    if "service" in message:
        print("DEBUG service field:", message['service'])
        if message['service'] == "/NotifyUserComponent/StartAlarm":
            predicates['alarm'] = True
            print("ALARM TRIGGERED")
    
    if "topic" in message and "battery" in message['topic']:
        battery_status =float(message.get('percentage', 100))
        predicates['high_battery'] = battery_status > 30
    
    if "topic" in message and "/monitoring_clock" in message['topic']:
        predicates['alarm'] = False

    print("predicates", predicates)
    print("keys:", list(message.keys()))
    #print("message", message)
    

    return predicates