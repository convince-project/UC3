# property to verify: wheels in hardware fault
"""
H(not wheel_hardware_fault)
"""
PROPERTY = r"historically(not {hardware_fault})"

# predicates used in the property (initialization for time 0)
predicates = dict(
    hardware_fault = False,
    time = 0,
)

# function to abstract a dictionary (obtained from Json message) into a list of predicates
def abstract_message(message):
    if message['time'] <= predicates['time']:
        predicates['time'] += 0.0000001
    else:
        predicates['time'] = message['time']

    # Controllo se almeno uno dei dati (escludendo i primi due) è uguale a 104
    if "topic" in message and "controlModes" in message['topic']:
        if "data" in message and len(message['data']) > 2:
            # Prendi tutti i valori dall'indice 2 in poi
            data_subset = message['data'][2:]
            print(f"Dati esclusi i primi due: {data_subset}")
            # Imposta True se almeno uno vale 104, altrimenti False
            found = any(value == 104 for value in data_subset)
            if found:
                print("Trovato hardware fault con valore 104")
            predicates['hardware_fault'] = bool(found)
        else:
            # nessun dato utile => non c'è fault
            predicates['hardware_fault'] = False

    print("predicates", predicates)
    print("message", message)

    return predicates

