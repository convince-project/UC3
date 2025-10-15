#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class MultiMonitor(Node):
    def __init__(self):
        super().__init__('multi_monitor')

        # Tabella di recap delle proprietà
        self.property_formulas = {
            'prop1': 'historically({high_battery} -> historically( not {alarm}))',
            'prop2': 'historically(({alarm} -> once{low_battery}) and not( not {alarm} since[5:] {low_battery}))',
            'prop3': 'historically(once[5:]{t} -> once[:5]{battery_published})',
            'prop5': 'historically(once[7:]{t} -> once[:7]{people_following_published})',
            'prop6': 'historically(once[5:]{t} -> once[:3]{camera_published})',
            'prop7': 'historically(not {critical_battery})',
            'prop8': '(not {wheel_hardware_fault})',
            'prop9': '(not {hardware_fault})',
            'prop10POI': 'historically( ({poi1_completed} -> {poi1_selected}) and ({poi1_selected} -> ( not( ((not {poi1_completed}) and {poi1_selected}) since[50:] {poi1_sel_start} ))))',
            'prop11': '({well_localized})',
            'prop12': 'TBD - da definire'
        }

        # Elenco dei topic da monitorare (modifica i nomi come servono)
        self.topics = [

            '/monitor_prop1/monitor_verdict',
            '/monitor_prop2/monitor_verdict',
            '/monitor_prop3/monitor_verdict',

            '/monitor_prop5/monitor_verdict',
            '/monitor_prop6/monitor_verdict',
            '/monitor_prop7/monitor_verdict',
            '/monitor_prop8/monitor_verdict',
            '/monitor_prop9/monitor_verdict',
            '/monitor_propPOI1/monitor_verdict',
            '/monitor_propPOI2/monitor_verdict',
            '/monitor_propPOI3/monitor_verdict',
            '/monitor_propPOI4/monitor_verdict',
            '/monitor_propPOI5/monitor_verdict',
            '/monitor_prop11/monitor_verdict',
            '/monitor_prop12/monitor_verdict',
            
        ]

        # Stato attuale di ogni topic
        self.status = {t: "unknown" for t in self.topics}
        
        # Timestamp dell'ultimo messaggio ricevuto per ogni topic
        self.last_message_time = {t: 0.0 for t in self.topics}
        
        # Timeout in secondi dopo il quale un topic è considerato "unknown"
        self.timeout_seconds = 3.0

        # Crea una sottoscrizione per ogni topic
        for topic in self.topics:
            self.create_subscription(String, topic,
                                     lambda msg, t=topic: self.callback(msg, t),
                                     10)

        # Timer per stampare lo stato ogni secondo
        self.timer = self.create_timer(1.0, self.print_status)
        
        # Flag per alternare tra tabella e stati
        self.show_table = True

    def callback(self, msg, topic_name):
        data = msg.data.strip().lower()
        if data not in ["currently_true", "currently_false"]:
            self.get_logger().warn(f"Topic {topic_name}: valore inatteso '{msg.data}'")
            return
        self.status[topic_name] = data
        self.last_message_time[topic_name] = time.time()

    def check_timeouts(self):
        """Controlla se alcuni topic non stanno più pubblicando"""
        current_time = time.time()
        for topic in self.topics:
            time_since_last_msg = current_time - self.last_message_time[topic]
            # Se è passato più tempo del timeout e il topic non è già unknown
            if time_since_last_msg > self.timeout_seconds and self.status[topic] != "unknown":
                if self.last_message_time[topic] > 0:  # Solo se aveva ricevuto almeno un messaggio
                    self.get_logger().warn(f"Topic {topic}: timeout - nessun messaggio da {time_since_last_msg:.1f}s")
                    self.status[topic] = "unknown"

    def print_property_table(self):
        print("\n" + "="*100)
        print("TABELLA RECAP PROPRIETÀ MONITORATE")
        print("="*100)
        for prop, formula in self.property_formulas.items():
            print(f"{prop:12s} --> {formula}")
        print("="*100 + "\n")

    def print_status(self):
        # Prima controlla i timeout
        self.check_timeouts()
        
        if self.show_table:
            # Stampa la tabella delle proprietà
            self.print_property_table()
        else:
            # Stampa lo stato dei topic
            all_true = all(v == "currently_true" for v in self.status.values())
            has_false = any(v == "currently_false" for v in self.status.values())
            has_unknown = any(v == "unknown" for v in self.status.values())
            
            print("\n=== STATUS CHECK ===")
            current_time = time.time()
            for topic, val in self.status.items():
                # Mostra anche da quanto tempo non arriva un messaggio
                if self.last_message_time[topic] > 0:
                    time_since_last = current_time - self.last_message_time[topic]
                    time_info = f"({time_since_last:.1f}s ago)"
                else:
                    time_info = "(mai ricevuto)"
                print(f"{topic:40s}: {val:15s} {time_info}")
            
            if all_true:
                print("✅  Tutti TRUE")
            elif has_false:
                print("❌  ERRORE: almeno un topic è currently_false")
            elif has_unknown:
                print("⚠️   WARNING: almeno un topic non pubblica (unknown)")
            
            print("====================\n")
        
        # Alterna per il prossimo ciclo
        self.show_table = not self.show_table


def main(args=None):
    rclpy.init(args=args)
    node = MultiMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Interruzione ricevuta, chiusura in corso...")
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass
        print("✅ Monitor chiuso correttamente")


if __name__ == '__main__':
    main()
