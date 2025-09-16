# property to verify
"""
H((P[5:] True) IMPLIES P[:5] camera_published)
"""

PROPERTY = r"historically(once[5:]{t} -> once[:5]{camera_published})"

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
    if message['topic'] in [
        "camera_left/image_rect_color",
        "camera_right/image_rect_color",
        "camera_rgbd/color/image_rect_color",
        "camera_rgbd/depth/image_rect",
        "camera/depth/color/points"
    ]:
        predicates['camera_published'] = True
    elif message['topic'] == "clock":
        predicates['camera_published'] = False
    print("predicates", predicates)
    return predicates
