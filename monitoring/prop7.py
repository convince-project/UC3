# property to verify
"""
H(not critical_battery)
"""
PROPERTY = r"historically(not {critical_battery})"

# predicates used in the property (initialization for time 0)
predicates = dict(
    critical_battery = False,
    time = 0,
)

# function to abstract a dictionary (obtained from Json message) into a list of predicates
def abstract_message(message):
    if message['time'] <= predicates['time']:
        predicates['time'] += 0.0000001
    else:
        predicates['time'] = message['time']

    if "topic" in message and "battery" in message['topic']:
        battery_status = float(message.get('percentage', 100))
        predicates['critical_battery'] = battery_status <= 10
        print("battery status", battery_status, "critical:", predicates['critical_battery'])
        
    if "topic" in message and "/moniitoring_clock" in message['topic']:
        predicates['alarm'] = False
    # print("predicates", predicates)
    # print("message", message)
    return predicates
