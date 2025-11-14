# property to verify
"""
H((P[5:] True) IMPLIES P[:5] battery_published)"""

PROPERTY = r"(once[5:]{t} -> once[:5]{battery_published})"


predicates = dict(

    battery_published = False,

    t = True,

    time = 0

)

def abstract_message(message):
    predicates['time'] = message['time']
    print("message", message)
    if message['topic'] == "/monitoring_clock":
        predicates['battery_published'] = False
    elif message['topic'] == "/battery_status":
        predicates['battery_published'] = True
    print("predicates", predicates)
    return predicates