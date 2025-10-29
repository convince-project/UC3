# property to verify
"""
H((P[5:] True) IMPLIES P[:3] connected_to_network)
"""

PROPERTY = r"(once[5:]{t} -> once[:3]{connected_to_network})"

predicates = dict(
    connected_to_network = False,
    t = True,
    time = 0
)

def abstract_message(message):
    if message['time'] <= predicates['time']:
        predicates['time'] += 0.0000001
    else:
        predicates['time'] = message['time']
    print("message", message)
    if message['topic'] in ["/CheckNetworkComponent/NetworkStatus"]:
        # Acced to the data field to determine if the camera is publishing
        predicates['connected_to_network'] = message['data']
        print(f" connected_to_network, data: { message['data']}")
    elif message['topic'] in ["/monitoring_clock"]:
        predicates['connected_to_network'] = False
        print("clock")
    return predicates
