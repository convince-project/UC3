"""
H(POI_1_selected => F[0:t] POI_1_completed)
"""
# property definition for POI 7 completion within time t
PROPERTY = r"historically({poi1_selected} -> eventually[0:x]{poi1_completed})"

# predicates used in the property (initialization for time 0)
predicates = dict(
    poi1_selected = False,
    poi1_completed = False,
    time = 0,
)

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
    if "service" in message and "response" in message:
        if message['service'] == "IsPoiDone1Skill/tick" and message['response']['status'] == 0:
            predicates['poi1_completed'] = True
        elif message['service'] == "IsPoiDone1Skill/tick" and message['response']['status'] != 0:
            predicates['poi1_completed'] = False

        if message['service'] == "SetPoi1Skill/tick" and message['response']['status'] == 0:
            predicates['poi1_selected'] = True
        elif message['service'] == "SetPoi1Skill/tick" and message['response']['status'] != 0:
            predicates['poi1_selected'] = False

    return predicates
