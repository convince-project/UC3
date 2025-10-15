#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import threading
try:
    import tkinter as tk
except Exception:
    tk = None

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

        # Lock per accesso thread-safe (rclpy callbacks vs GUI thread)
        self.lock = threading.Lock()

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
        with self.lock:
            self.status[topic_name] = data
            self.last_message_time[topic_name] = time.time()

    def check_timeouts(self):
        """Controlla se alcuni topic non stanno più pubblicando"""
        current_time = time.time()
        with self.lock:
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
            # Copia sicura dello stato per stampa
            with self.lock:
                status_copy = dict(self.status)
                last_copy = dict(self.last_message_time)

            for topic, val in status_copy.items():
                # Mostra anche da quanto tempo non arriva un messaggio
                if last_copy[topic] > 0:
                    time_since_last = current_time - last_copy[topic]
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

    # If tkinter is not available, fallback to headless behavior
    if tk is None:
        print("Tkinter non disponibile: avvio in modalità testuale")
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
        return

    # GUI is available: run ROS spin in background thread and GUI in main thread
    class MonitorGUI:
        def __init__(self, node, cols=4, square_size=80, refresh_ms=500):
            self.node = node
            self.cols = cols
            self.square_size = square_size
            self.refresh_ms = refresh_ms
            self.root = tk.Tk()
            self.root.title('MultiMonitor GUI')

            # Property table at the top (read-only)
            self.top_frame = tk.Frame(self.root, padx=10, pady=5)
            self.top_frame.pack(fill='x')
            tbl_label = tk.Label(self.top_frame, text='Property Table', font=('TkDefaultFont', 10, 'bold'))
            tbl_label.pack(anchor='w')
            tbl_text = tk.Text(self.top_frame, height=8, width=100, wrap='none')
            tbl_text.pack(fill='x')
            # Build table text from node.property_formulas
            lines = []
            for prop, formula in self.node.property_formulas.items():
                lines.append(f"{prop:12s} --> {formula}")
            tbl_text.insert('1.0', "\n".join(lines))
            tbl_text.config(state='disabled')

            self.frame = tk.Frame(self.root, padx=10, pady=10)
            self.frame.pack()
            self.topic_widgets = {}
            self._build_grid()
            # on close
            self.root.protocol('WM_DELETE_WINDOW', self.on_close)

        def _build_grid(self):
            for idx, topic in enumerate(self.node.topics):
                r = idx // self.cols
                c = idx % self.cols
                # Friendly label
                name = topic.strip('/').replace('/', '\\n')
                cell = tk.Frame(self.frame, width=self.square_size, height=self.square_size)
                cell.grid(row=r*2, column=c, padx=8, pady=8)
                cell.grid_propagate(False)
                color_label = tk.Label(cell, bg='grey', width=10, height=5, relief='ridge')
                color_label.pack(expand=True, fill='both')
                text = tk.Label(self.frame, text=name)
                text.grid(row=r*2+1, column=c)
                self.topic_widgets[topic] = color_label

        def update_ui(self):
            with self.node.lock:
                status_copy = dict(self.node.status)
            for topic, widget in self.topic_widgets.items():
                val = status_copy.get(topic, 'unknown')
                if val == 'currently_true':
                    color = 'green'
                elif val == 'currently_false':
                    color = 'red'
                else:
                    color = 'grey'
                widget.config(bg=color)
            self.root.after(self.refresh_ms, self.update_ui)

        def on_close(self):
            # Shutdown ROS and close
            try:
                self.root.destroy()
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass

        def run(self):
            # start periodic updates
            self.root.after(0, self.update_ui)
            self.root.mainloop()

    # Start rclpy spin in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    gui = MonitorGUI(node)
    try:
        gui.run()
    except KeyboardInterrupt:
        print("\n🛑 Interruzione ricevuta, chiusura in corso...")
    finally:
        # Ensure shutdown
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        # give spin thread a moment
        spin_thread.join(timeout=1.0)
        print("✅ Monitor chiuso correttamente")


if __name__ == '__main__':
    main()
