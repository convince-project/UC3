"""
H(POI_2_selected => F[0:t] POI_2_completed)
"""
# property definition for POI 2 completion within time t
PROPERTY = r"historically(({poi2_completed} -> once{poi2_selected}) and not( not {poi2_completed} since[50:] {poi2_selected}))"


# predicates used in the property (initialization for time 0)
predicates = dict(
    poi2_selected = False,
    poi2_completed = False,
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

    if message.get('topic') == "/monitoring_clock":
        return predicates

    if "topic" in message and "GetCurrentPoi" in message['topic']:
        if "response" in message:
            for resp in message["response"]:
                if resp.get("poi_number") == 2:
                   predicates['poi2_selected'] = True

    if "topic" in message and "GetInt" in message['topic']:
        if "response" in message:
            for resp in message["response"]:
                field_name = resp.get("field_name")
                if field_name == "PoiDone2":
                    if resp.get("value") == 1:
                        predicates['poi2_completed'] = True
                    else:
                        predicates['poi2_completed'] = False

    if "topic" in message and "Reset" in message['topic']:
        predicates['poi2_selected'] = False
        predicates['poi2_completed'] = False

    return predicates
