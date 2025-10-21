# property to verify
"""
H((P[5:] True) IMPLIES P[:3] camera_published)
"""

PROPERTY = r"historically(once[5:]{t} -> once[:3]{camera_published})"

predicates = dict(
    camera_published = False,
    t = True,
    time = 0
)

def abstract_message(message):
    if message['time'] <= predicates['time']:
        predicates['time'] += 0.0000001
    else:
        predicates['time'] = message['time']
    print("message", message)
    if message['topic'] in ["/cer_camera_repeater/publisher_status"]:
        # Acced to the data field to determine if the camera is publishing
        data_value = message['data']
        if data_value > 0:
            predicates['camera_published'] = True
        else:
            predicates['camera_published'] = False
        print(f"camera published, data: {data_value}")
    elif message['topic'] in ["/monitoring_clock"]:
        predicates['camera_published'] = False
        print("clock")
    return predicates
