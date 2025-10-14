"""
(not critical_battery)
"""

PROPERTY = r"(not {low_covariance})"

# predicates used in the property (initialization for time 0)

# in here we can add all the predicates we are interested in.. Of course, we also need to define how to translate Json messages to predicates.

# function to abstract a dictionary (obtained from Json message) into a list of predicates

predicates = dict(

    time = 0,

    low_covariance = False,
)

# in here we can add all the predicates we are interested in.. Of course, we also need to define how to translate Json messages to predicates.

# function to abstract a dictionary (obtained from Json message) into a list of predicates

def abstract_message(message):
    if message['time'] <= predicates['time']:
        predicates['time'] += 0.0000001
    else:
        predicates['time'] = message['time']
    
    if "topic" in message and "localization" in message['topic']:
        cov_value = message['data']
        predicates['low_covariance'] = cov_value < 10

    print("predicates", predicates)
    print("message", message)

    return predicates

# property to verify
# Ogni volta che si attiva l’allarme (alarm), in passato la batteria è stata almeno una volta sotto il 30% (low_battery).
# Inoltre, non deve accadere che, dopo l’attivazione dell’allarme, per almeno 5 unità di tempo non si sia verificato che la batteria era sotto il 30%."""