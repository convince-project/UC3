# property to verify
"""
H((P[5:] True) IMPLIES P[:5] people_following_published)"""

PROPERTY = r"historically(once[7:]{t} -> once[:7]{people_following_published})"

# predicates used in the property (initialization for time 0)

# in here we can add all the predicates we are interested in.. Of course, we also need to define how to translate Json messages to predicates.

# function to abstract a dictionary (obtained from Json message) into a list of predicates

predicates = dict(

    people_following_published = False,

    t = True,

    time = 0

)

# in here we can add all the predicates we are interested in.. Of course, we also need to define how to translate Json messages to predicates.

# function to abstract a dictionary (obtained from Json message) into a list of predicates

def abstract_message(message):
    predicates['time'] = message['time']
    print("message", message)
    if message['topic'] == "/monitoring_clock":
        predicates['people_following_published'] = False
    elif message['topic'] == "/PeopleDetectorFilterComponent/filtered_detection":
        predicates['people_following_published'] = True
    print("predicates", predicates)
    return predicates