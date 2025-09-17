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
    #print("camera_received, topic ", message['topic'])
    if message['topic'] in ["/cer/realsense_repeater/color_image"]:
        predicates['camera_published'] = True
        print("camera published")
    elif message['topic'] in ["/monitoring_clock"]:
        predicates['camera_published'] = False
        print("clock")
    return predicates
