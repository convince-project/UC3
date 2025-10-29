#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import threading
import re
import signal
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
            'prop3': '(once[5:]{t} -> once[:5]{battery_published})',
            'prop4': 'historically(({poi1_selected} -> once( not {poi1_completed})) and not( not {poi1_selected} since [50:] not {poi1_completed}))',
            'prop5': '(once[7:]{t} -> once[:7]{people_following_published})',
            'prop6': '(once[5:]{t} -> once[:3]{camera_published})',
            'prop7': '(not {critical_battery})',
            'prop8': '(not {wheel_hardware_fault})',
            'prop9': '(not {hardware_fault})',
            'prop10POI': 'historically( ({poi1_completed} -> {poi1_selected}) and ({poi1_selected} -> ( not( ((not {poi1_completed}) and {poi1_selected}) since[50:] {poi1_sel_start} ))))',
            'prop11': '({well_localized})',
            'prop12': '( {is_recording} -> (not {is_unplagged}) )',
            'prop13': '(once[5:]{t} -> once[:3]{connected_to_network})',
            'prop14': '(once[5:]{t} -> once[:3]{connected_to_web})'
        }

        # Elenco dei topic da monitorare (modifica i nomi come servono)
        self.topics = [

            '/monitor_prop1/monitor_verdict',
            '/monitor_prop2/monitor_verdict',
            '/monitor_prop3/monitor_verdict',
            '/monitor_prop4/monitor_verdict',
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
            '/monitor_prop13/monitor_verdict',
            '/monitor_prop14/monitor_verdict'            
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
    # holder for GUI reference so signal handler can close it
    gui_holder = {'gui': None}

    # Register signal handlers so external managers (e.g. yarpmanager) can stop this process
    # gracefully by sending SIGTERM/SIGINT instead of forcing a kill.
    def _graceful_shutdown(signum, frame):
        try:
            node.get_logger().info(f"Signal {signum} received, shutting down...")
        except Exception:
            pass
        # try to close GUI if running
        try:
            g = gui_holder.get('gui')
            if g is not None:
                try:
                    g.root.quit()
                except Exception:
                    pass
        except Exception:
            pass
        # Destroy ROS node and shutdown rclpy
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    signal.signal(signal.SIGTERM, _graceful_shutdown)
    signal.signal(signal.SIGINT, _graceful_shutdown)

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
            self.root.title('MultiMonitor')
            # improve default look
            try:
                self.root.tk.call('tk', 'scaling', 1.2)
            except Exception:
                pass

            # overall styles
            default_font = ('Segoe UI', 10)
            bold_font = ('Segoe UI', 10, 'bold')

            # Property table at the top (read-only) with monospace font and scrollbar
            self.top_frame = tk.Frame(self.root, padx=12, pady=8, bg='#f5f7fa')
            self.top_frame.pack(fill='x')
            tbl_label = tk.Label(self.top_frame, text='Property Table', font=bold_font, bg='#f5f7fa')
            tbl_label.pack(anchor='w')
            # text with vertical scrollbar
            text_container = tk.Frame(self.top_frame)
            text_container.pack(fill='x')
            tbl_scroll = tk.Scrollbar(text_container)
            tbl_text = tk.Text(text_container, height=8, wrap='none', font=('Consolas', 10), bd=1, relief='solid')
            tbl_scroll.pack(side='right', fill='y')
            tbl_text.pack(side='left', fill='x', expand=True)
            tbl_scroll.config(command=tbl_text.yview)
            tbl_text.config(yscrollcommand=tbl_scroll.set)
            # Build table text from node.property_formulas
            lines = []
            for prop, formula in self.node.property_formulas.items():
                lines.append(f"{prop:12s} --> {formula}")
            tbl_text.insert('1.0', "\n".join(lines))
            tbl_text.config(state='disabled')

            # main grid for topic tiles
            self.frame = tk.Frame(self.root, padx=12, pady=12)
            self.frame.pack(fill='both', expand=True)
            self.topic_widgets = {}
            self._build_grid()
            # legend and controls
            legend = tk.Frame(self.root, pady=6)
            legend.pack(fill='x')
            tk.Label(legend, text='Legend:', font=bold_font).pack(side='left', padx=(8,4))
            tk.Label(legend, text=' ', bg='green', width=2, relief='ridge').pack(side='left', padx=4)
            tk.Label(legend, text='currently_true').pack(side='left', padx=(0,12))
            tk.Label(legend, text=' ', bg='red', width=2, relief='ridge').pack(side='left', padx=4)
            tk.Label(legend, text='currently_false').pack(side='left', padx=(0,12))
            tk.Label(legend, text=' ', bg='grey', width=2, relief='ridge').pack(side='left', padx=4)
            tk.Label(legend, text='not published').pack(side='left', padx=(0,8))
            # on close
            self.root.protocol('WM_DELETE_WINDOW', self.on_close)

        def _build_grid(self):
            for idx, topic in enumerate(self.node.topics):
                r = idx // self.cols
                c = idx % self.cols
                # Friendly label: show only the property number or short id under the square
                # Examples: '/monitor_prop1/monitor_verdict' -> '1',
                # '/monitor_propPOI1/monitor_verdict' -> 'POI1'
                topic_clean = topic.strip('/')
                name = None
                m = re.search(r'prop(?:erty)?(?:_)?(poi\d+|POI\d+|[A-Za-z0-9]+)', topic_clean, re.IGNORECASE)
                if m:
                    # group could be like '1', 'POI1', 'POI2', '10POI' etc.
                    grp = m.group(1)
                    # if it's numeric (e.g., '1') keep as is, else upper-case POI
                    name = grp.upper() if any(ch.isalpha() for ch in grp) else grp
                else:
                    # fallback: try to extract digits from the topic
                    dm = re.search(r'(\d+)', topic_clean)
                    if dm:
                        name = dm.group(1)
                    else:
                        name = str(idx+1)
                cell = tk.Frame(self.frame, width=self.square_size, height=self.square_size, bd=1, relief='groove')
                cell.grid(row=r*2, column=c, padx=8, pady=8, sticky='nsew')
                cell.grid_propagate(False)
                # color block
                color_label = tk.Label(cell, bg='grey', width=10, height=5, relief='ridge')
                color_label.pack(expand=True, fill='both', padx=4, pady=4)
                # topic property number / short id
                text = tk.Label(self.frame, text=name, wraplength=self.square_size+20, justify='center')
                text.grid(row=r*2+1, column=c)
                # last update label
                time_label = tk.Label(self.frame, text='(mai ricevuto)', font=('Segoe UI', 8), fg='#555555')
                time_label.grid(row=r*2+2, column=c)
                self.topic_widgets[topic] = (color_label, time_label)

            # allow grid to expand
            for col in range(self.cols):
                self.frame.grid_columnconfigure(col, weight=1)
            rows = (len(self.node.topics) + self.cols - 1) // self.cols
            for row in range(rows*3):
                self.frame.grid_rowconfigure(row, weight=1)

        def update_ui(self):
            with self.node.lock:
                status_copy = dict(self.node.status)
            current_time = time.time()
            for topic, widgets in self.topic_widgets.items():
                val = status_copy.get(topic, 'unknown')
                color = 'grey'
                if val == 'currently_true':
                    color = '#2ecc71'  # pleasant green
                elif val == 'currently_false':
                    color = '#e74c3c'  # soft red
                else:
                    color = '#95a5a6'  # grey
                # widgets is (color_label, time_label)
                color_label, time_label = widgets
                color_label.config(bg=color)
                # update timestamp
                last = self.node.last_message_time.get(topic, 0)
                if last > 0:
                    dt = current_time - last
                    time_label.config(text=f"{dt:.1f}s ago")
                else:
                    time_label.config(text='(mai ricevuto)')
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

    # Start rclpy spin in background thread (non-daemon so we can join on shutdown)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=False)
    spin_thread.start()

    gui = MonitorGUI(node)
    # expose gui to signal handler so it can be closed externally
    gui_holder['gui'] = gui
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
