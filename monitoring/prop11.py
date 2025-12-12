"""
(not critical_battery)
"""

PROPERTY = r"({well_localized})"

# predicates used in the property (initialization for time 0)

# in here we can add all the predicates we are interested in.. Of course, we also need to define how to translate Json messages to predicates.

# function to abstract a dictionary (obtained from Json message) into a list of predicates

predicates = dict(

    time = 0,

    well_localized = True,
)

# in here we can add all the predicates we are interested in.. Of course, we also need to define how to translate Json messages to predicates.

# function to abstract a dictionary (obtained from Json message) into a list of predicates

def abstract_message(message):
    if message['time'] <= predicates['time']:
        predicates['time'] += 0.0000001
    else:
        predicates['time'] = message['time']

    print("predicates", predicates)
    print("message", message)
    
    if "topic" in message and "/amcl_pose" in message['topic']:
        cov_list = message['pose']['covariance']
        print("Covariance list:", cov_list)
        
        # Calculate absolute values and find maximum
        abs_values = [abs(val) for val in cov_list]
        max_abs_value = max(abs_values)
        print("Maximum absolute covariance value:", max_abs_value)
        
        # Set well_localized based on maximum absolute value
        predicates['well_localized'] = max_abs_value <= 0.01

    print("predicates", predicates)
    print("message", message)

    return predicates

# property to verify
# Ogni volta che si attiva l’allarme (alarm), in passato la batteria è stata almeno una volta sotto il 30% (low_battery).
# Inoltre, non deve accadere che, dopo l’attivazione dell’allarme, per almeno 5 unità di tempo non si sia verificato che la batteria era sotto il 30%."""
