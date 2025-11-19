"""
H(POI_3_selected => F[0:t] POI_3_completed)
"""
# property definition for POI 3 completion within time t
PROPERTY = r"historically( ({poi3_completed} -> {poi3_selected}) and ({poi3_selected} -> ( not( ((not {poi3_completed}) and {poi3_selected}) since[36000:] {poi3_sel_start} ))))"

predicates = dict(
    poi3_selected=False,
    poi3_completed=False,
    poi3_sel_start=False,     # evento istantaneo
    _poi3_selected_prev=False,# ausiliario interno
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
    predicates['poi3_sel_start'] = False

    if message.get('topic') == "/monitoring_clock":
        return predicates

    
    # default: non selezionato, poi settiamo True se il POI corrente è 3
    current_selected = False

    if "topic" in message and "GetCurrentPoi" in message['topic']:
        if "response" in message:
            for resp in message["response"]:
                if resp.get("poi_number") == 3:
                    current_selected = True
                    predicates['poi3_selected'] = True

    # fronte di salita: ora True, prima False
    if current_selected and not predicates['_poi3_selected_prev']:
        predicates['poi3_sel_start'] = True

    
    predicates['_poi3_selected_prev'] = current_selected

    # completamento
    if "topic" in message and "GetInt" in message['topic']:
        if "response" in message:
            for resp in message["response"]:
                if resp.get("field_name") == "PoiDone3":
                    predicates['poi3_completed'] = (resp.get("value") == 1)

    # reset tour (se ce l’hai): azzera stati e il “prev”
    if "topic" in message and "Reset" in message['topic']:
        predicates['poi3_selected'] = False
        predicates['_poi3_selected_prev'] = False
        predicates['poi3_completed'] = False
        # opzionale: puoi anche emettere un evento di reset se lo vuoi usare altrove

    return predicates
