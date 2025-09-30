# property to verify

'''   H(POI_1_selected => P -POI_1_completed) AND - (-POI_1_selected S[2 : ] -POI_1_completed)
'''
PROPERTY = r"historically(({poi1_selected} -> once( not {poi1_completed})) and not( not {poi1_selected} since [5:] not {poi1_completed}))"

# predicates used in the property (initialization for time 0)

# in here we can add all the predicates we are interested in.. Of course, we also need to define how to translate Json messages to predicates.

# function to abstract a dictionary (obtained from Json message) into a list of predicates

predicates = dict(

    poi1_selected = False,

    poi1_completed = False,

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
    if "topic" in message and "GetCurrentPoi" in message:
        if message['response']['poi_number'] == 1:
            predicates['poi1_selected'] = True
    if "topic" in message and "GetInt" in message:
        if message["request"]["field_name"]== "PoiDone1" and message['response']['value'] == 1:
            predicates['poi1_completed'] = True

    # predicates['service'] = True if 'service' in message else False

    # predicates['low_percentage'] = True if 'percentage' in message and message['percentage'] < 30 else False


    return predicates