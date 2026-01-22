# property to verify

'''   H(POI_1_selected => P -POI_1_completed) AND - (-POI_1_selected S[2 : ] -POI_1_completed)
'''
PROPERTY = r"historically( not( not {arrivedAtCS} since [300:] {alarm}))"

# predicates used in the property (initialization for time 0)

# in here we can add all the predicates we are interested in.. Of course, we also need to define how to translate Json messages to predicates.

# function to abstract a dictionary (obtained from Json message) into a list of predicates

predicates = dict(

    alarm = False,

    arrivedAtCS = False,

    time = 0,

)

# in here we can add all the predicates we are interested in.. Of course, we also need to define how to translate Json messages to predicates.

# function to abstract a dictionary (obtained from Json message) into a list of predicates

def abstract_message(message):

    if message['time'] <= predicates['time']:
        predicates['time'] += 0.0000001
    else:
        predicates['time'] = message['time']

    print("message", message)
    print("predicates", predicates)

    # int8 SKILL_SUCCESS=0
    # int8 SKILL_FAILURE=1
    # int8 SKILL_RUNNING=2
    if "topic" in message and "StartAlarm" in message['topic']:
        predicates['alarm'] = True

    if "topic" in message and "CheckNearToPoi" in message['topic']:
        if "response" in message:
            for resp in message["response"]:
                if resp.get("poi_name") == 'charging_station':
                    print("inside charging station")
                    predicates['arrivedAtCS'] = resp.get("is_near", False)

    return predicates
