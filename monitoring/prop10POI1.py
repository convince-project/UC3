"""
H(POI_1_selected => F[0:t] POI_1_completed)
"""
# property definition for POI 1 completion within time t
PROPERTY = r"historically( ({poi1_completed} -> {poi1_selected}) and ({poi1_selected} -> ( not( ((not {poi1_completed}) and {poi1_selected}) since[36000:] {poi1_sel_start} ))))"

predicates = dict(
    poi1_selected=False,
    poi1_completed=False,
    poi1_sel_start=False,     # evento istantaneo
    _poi1_selected_prev=False,# ausiliario interno
    time=0,
)

def abstract_message(message):
    # tempo monotono
    if message['time'] <= predicates['time']:
        predicates['time'] += 1e-7
    else:
        predicates['time'] = message['time']

    print("message:", message)
    print("predicates:", predicates)
    
    # reset eventi istantanei a ogni messaggio
    predicates['poi1_sel_start'] = False

    if message.get('topic') == "/monitoring_clock":
        return predicates

    
    # default: non selezionato, poi settiamo True se il POI corrente è 1
    current_selected = False

    if "topic" in message and "GetCurrentPoi" in message['topic']:
        if "response" in message:
            for resp in message["response"]:
                if resp.get("poi_number") == 1:
                    current_selected = True
                    predicates['poi1_selected'] = True

    # fronte di salita: ora True, prima False
    if current_selected and not predicates['_poi1_selected_prev']:
        predicates['poi1_sel_start'] = True

    predicates['_poi1_selected_prev'] = current_selected

    # completamento
    if "topic" in message and "GetInt" in message['topic']:
        if "response" in message:
            for resp in message["response"]:
                if resp.get("field_name") == "PoiDone1":
                    predicates['poi1_completed'] = (resp.get("value") == 1)

    # reset tour (se ce l’hai): azzera stati e il “prev”
    if "topic" in message and "Reset" in message['topic']:
        predicates['poi1_selected'] = False
        predicates['_poi1_selected_prev'] = False
        predicates['poi1_completed'] = False
        # opzionale: puoi anche emettere un evento di reset se lo vuoi usare altrove

    return predicates
