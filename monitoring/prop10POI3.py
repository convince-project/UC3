"""
H(POI_3_selected => F[0:t] POI_3_completed)
"""
# property definition for POI 3 completion within time t
PROPERTY = r"historically(({poi3_completed} -> once{poi3_selected}) and not( not {poi3_completed} since[50:] {poi3_selected}))"


# predicates used in the property (initialization for time 0)
predicates = dict(
    poi3_selected = False,
    poi3_completed = False,
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

    if message['topic'] == "/monitoring_clock":
        return predicates

    if "topic" in message and "GetCurrentPoi" in message['topic']:
        if "response" in message:
            for resp in message["response"]:
                if resp.get("poi_number") == 3:
                   predicates['poi3_selected'] = True
                        
    if "topic" in message and "GetInt" in message['topic']:
        if "response" in message:
            for resp in message["response"]:
                field_name = resp.get("field_name")
                if field_name == "PoiDone3":
                    if resp.get("value") == 1:
                        predicates['poi3_completed'] = True
                    else:
                        predicates['poi3_completed'] = False
    
    if "topic" in message and "Reset" in message['topic']:
        predicates['poi3_selected'] = False
        predicates['poi3_completed'] = False

    return predicates
