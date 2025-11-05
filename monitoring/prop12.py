"""
(is_recording IMPLIES not (is_unplagged))
"""

# Property: always, if the microphone is recording then is_unplagged must be false
PROPERTY = r"( {is_recording} -> (not {is_unplagged}) )"

# predicates used in the property (initialization for time 0)

# in here we can add all the predicates we are interested in.. Of course, we also need to define how to translate Json messages to predicates.

# function to abstract a dictionary (obtained from Json message) into a list of predicates

predicates = dict(

    time = 0,

    is_recording = False,

    is_unplagged = False
)

# in here we can add all the predicates we are interested in.. Of course, we also need to define how to translate Json messages to predicates.

# function to abstract a dictionary (obtained from Json message) into a list of predicates

def abstract_message(message):
    if message['time'] <= predicates['time']:
        predicates['time'] += 0.0000001
    else:
        predicates['time'] = message['time']
    
    # if the message is from an audio topic, compute stddev of the data
    if "topic" in message and "status" in message['topic']:
        # update is_recording from message status: 'ok' -> True, otherwise False
        # print("message", message)
        if message['layout']['data_offset'] == 0:
            # print("predicates", predicates)
            predicates['is_recording'] = True
        else:
            predicates['is_recording'] = False

    if predicates.get('is_recording'):

        if "topic" in message and "sound_array" in message['topic']:
            import statistics
            #print("message", message)

            data = message.get('data')

            # Normalize data to a list of numbers
            if data is None:
                nums = []
            elif isinstance(data, (list, tuple)):
                nums = list(data)
            else:
                # scalar value -> single-element list
                nums = [data]

            # compute population stddev when possible; handle edge cases
            try:
                if len(nums) == 0:
                    stddev = 0.0
                else:
                    # convert to floats defensively
                    nums_f = [float(x) for x in nums]
                    stddev = statistics.pstdev(nums_f)
                    print("stddev:", stddev)
            except Exception:
                # fallback: if conversion fails, mark as not unplagged
                stddev = float('inf')

            predicates['is_unplagged'] = (stddev < 10)


    print("predicates", predicates)
    #print("message", message)

    return predicates

# property to verify
# Ogni volta che si attiva l’allarme (alarm), in passato la batteria è stata almeno una volta sotto il 30% (low_battery).
# Inoltre, non deve accadere che, dopo l’attivazione dell’allarme, per almeno 5 unità di tempo non si sia verificato che la batteria era sotto il 30%."""