"""
H(POI_1_selected => F[0:t] POI_1_completed)
"""
# property definition for POI 7 completion within time t
PROPERTY = r"historically( not( not {po1_completed} since[50:] {poi1_selected}))"


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

    # print("message", message)
    print("predicates", predicates)

    if message['topic'] == "/monitoring_clock":
        return predicates
    # Global map to match requests and responses through the sequence_number

    # int8 SKILL_SUCCESS=0
    # int8 SKILL_FAILURE=1
    # int8 SKILL_RUNNING=2
    if "topic" in message and "GetCurrentPoi" in message['topic']:
        print("message", message)
        if "response" in message:
            for resp in message["response"]:
                if resp.get("poi_number") == 1:
                   predicates['poi1_selected'] = True
                        
    if "topic" in message and "GetInt" in message['topic']:
        print("in if GetInt")
        if "response" in message:
            for resp in message["response"]:
                field_name = resp.get("field_name")
                print("message", message)
                if field_name == "PoiDone1":
                    if resp.get("error_msg") == "":
                        predicates['poi1_completed'] = True
                        # print("predicates", predicates)
                    else:
                        predicates['poi1_completed'] = False
                    # Rimuovi la richiesta accoppiata
    # predicates['service'] = True if 'service' in message else False

    # predicates['low_percentage'] = True if 'percentage' in message and message['percentage'] < 30 else Fals

    return predicates
