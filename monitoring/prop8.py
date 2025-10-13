# property to verify: wheels in hardware fault
"""
H(not wheel_hardware_fault)
"""
PROPERTY = r"(not {wheel_hardware_fault})"

# predicates used in the property (initialization for time 0)
predicates = dict(
    wheel_hardware_fault = False,
    time = 0,
)

# function to abstract a dictionary (obtained from Json message) into a list of predicates
def abstract_message(message):
    if message['time'] <= predicates['time']:
        predicates['time'] += 0.0000001
    else:
        predicates['time'] = message['time']


    # Accesso ai primi due dati dell'array
    if "topic" in message and "controlModes" in message['topic']:
        if "data" in message and len(message['data']) >= 2:
            first_data = message['data'][0]
            second_data = message['data'][1]
            print(f"Primi due dati: {first_data}, {second_data}")
        
            # Esempio: usa i primi due dati per impostare wheel_hardware_fault
            # Puoi modificare questa logica secondo le tue necessità
            predicates['wheel_hardware_fault'] = first_data == 104 or second_data == 104

    print("predicates", predicates)
    print("message", message)

    return predicates

