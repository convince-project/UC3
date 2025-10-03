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

    # Global map to match requests and responses through the sequence_number
    if not hasattr(abstract_message, "pending_requests"):
        abstract_message.pending_requests = {}

    # int8 SKILL_SUCCESS=0
    # int8 SKILL_FAILURE=1
    # int8 SKILL_RUNNING=2
    if "topic" in message and "GetCurrentPoi" in message['topic']:
        sequence_number = None
        if "info" in message and isinstance(message["info"], dict):
            sequence_number = message["info"].get("sequence_number")
        # If there is a request, save the sequence_number
        if "request" in message and isinstance(message["request"], list) and sequence_number is not None:
            abstract_message.pending_requests["poi_selected_" + str(sequence_number)] = True
        # If there is a response, check if the request matches through sequence_number
        if "response" in message and isinstance(message["response"], list) and sequence_number is not None:
            for resp in message["response"]:
                if resp.get("poi_number") == 1:
                    key = "poi_selected_" + str(sequence_number)
                    if abstract_message.pending_requests.get(key):
                        predicates['poi1_selected'] = True
                        del abstract_message.pending_requests[key]
    if "topic" in message and "GetInt" in message['topic']:
        print("in if GetInt")
        sequence_number = None
        if "info" in message and isinstance(message["info"], dict):
            sequence_number = message["info"].get("sequence_number")
        # If there is a request, save the associated field_name with the sequence_number
        if "request" in message and isinstance(message["request"], list) and sequence_number is not None:
            for req in message["request"]:
                print("in for", req)
                if req.get("field_name") == "PoiDone1":
                    print("field name found")
                    abstract_message.pending_requests[sequence_number] = "PoiDone1"
        # If there is a response, check if value==0 and the request matches through sequence_number
        if "response" in message and isinstance(message["response"], list) and sequence_number is not None:
            for resp in message["response"]:
                if resp.get("value") == 0:
                    print("in get value")
                    field_name = abstract_message.pending_requests.get(sequence_number)
                    print("field_name", field_name)
                    if field_name == "PoiDone1":
                        predicates['poi1_completed'] = True
                        # Remove the entry to avoid memory growth
                        del abstract_message.pending_requests[sequence_number]

    # predicates['service'] = True if 'service' in message else False

    # predicates['low_percentage'] = True if 'percentage' in message and message['percentage'] < 30 else False


    return predicates