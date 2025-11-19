# property to verify
"""
H((P[5:] True) IMPLIES P[:3] connected_to_web)
"""

PROPERTY = r"(once[5:]{t} -> once[:3]{connected_to_web})"

predicates = dict(
    connected_to_web = False,
    t = True,
    time = 0
)

def abstract_message(message):
    if message['time'] <= predicates['time']:
        predicates['time'] += 0.0000001
    else:
        predicates['time'] = message['time']
    print("message", message)
    if message['topic'] in ["/CheckNetworkComponent/WebStatus"]:
        # Acced to the data field to determine if the camera is publishing
        predicates['connected_to_web'] = message['data']
        print(f" connected_to_web, data: { message['data']}")
    elif message['topic'] in ["/monitoring_clock"]:
        predicates['connected_to_web'] = False
        print("clock")
    return predicates
