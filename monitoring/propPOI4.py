"""
H(POI_4_selected => F[0:t] POI_4_completed)
"""
# property definition for POI 4 completion within time t
PROPERTY = r"historically(({poi4_completed} -> once{poi4_selected}) and not( not {poi4_completed} since[50:] {poi4_selected}))"


# predicates used in the property (initialization for time 0)
predicates = dict(
    poi4_selected = False,
    poi4_completed = False,
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
                if resp.get("poi_number") == 4:
                   predicates['poi4_selected'] = True
                        
    if "topic" in message and "GetInt" in message['topic']:
        if "response" in message:
            for resp in message["response"]:
                field_name = resp.get("field_name")
                if field_name == "PoiDone4":
                    if resp.get("value") == 1:
                        predicates['poi4_completed'] = True
                    else:
                        predicates['poi4_completed'] = False
    
    if "topic" in message and "Reset" in message['topic']:
        predicates['poi4_selected'] = False
        predicates['poi4_completed'] = False

    return predicates
