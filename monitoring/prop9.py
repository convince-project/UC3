# property to verify
"""
H((cant_answer_question) -> (low_confidence))
"""
PROPERTY = r"historically({cant_answer_question} -> {low_confidence})"

# predicates used in the property (initialization for time 0)
predicates = dict(
    cant_answer_question = False,
    low_confidence = False,
    time = 0,
)

# function to abstract a dictionary (obtained from Json message) into a list of predicates
def abstract_message(message):
    if message['time'] <= predicates['time']:
        predicates['time'] += 0.0000001
    else:
        predicates['time'] = message['time']

    if message.get('topic') == "/DialogComponent/WaitForInteractionAction":
        # Supponiamo che il messaggio abbia solo 'confidence'
        confidence = message['data']
        predicates['low_confidence'] = confidence < 0.2 # soglia modificabile

    print("predicates", predicates)
    print("message", message)
    return predicates


# come funziona? in che modo viene modificato cant'answer question? il mesaggio mi passa entrambi? o me ne passa solo uno?s
