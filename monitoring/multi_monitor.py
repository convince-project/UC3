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

        # PROPERTIES RECAP TABLE
        # ------  Put here your real properties ------
        self.property_formulas = {
            'prop1': 'historically({high_battery} -> historically( not {alarm}))',
            'prop2': 'historically(({alarm} -> once{low_battery}) and not( not {alarm} since[5:] {low_battery}))',
            'prop3': '(once[5:]{t} -> once[:5]{battery_published})',
            'prop4': 'historically( not( not {startReached} since [600:] {tourFinished}))',
            'prop5': '(once[7:]{t} -> once[:7]{people_following_published})',
            'prop6': '(once[5:]{t} -> once[:3]{camera_published})',
            'prop7': '(not {critical_battery})',
            'prop8': '(not {wheel_hardware_fault})',
            'prop9': '(not {hardware_fault})',
            'prop10POI': 'historically( ({poi1_completed} -> {poi1_selected}) and ({poi1_selected} -> ( not( ((not {poi1_completed}) and {poi1_selected}) since[50:] {poi1_sel_start} ))))',
            'prop11': '({well_localized})',
            'prop12': '( {is_recording} -> (not {is_unplagged}) )',
            'prop13': '(once[5:]{t} -> once[:3]{connected_to_network})',
            'prop14': '(once[5:]{t} -> once[:3]{connected_to_web})',
            'prop15': 'historically( not( not {arrivedAtCS} since [600:] {alarm}))'
            
        }

        # List of topics to monitor
        # ------ Put here your real topics to monitor ------
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
            '/monitor_prop10POI1/monitor_verdict',
            '/monitor_prop10POI2/monitor_verdict',
            '/monitor_prop10POI3/monitor_verdict',
            '/monitor_prop10POI4/monitor_verdict',
            '/monitor_prop10POI5/monitor_verdict',
            '/monitor_prop11/monitor_verdict',
            '/monitor_prop12/monitor_verdict',
            '/monitor_prop13/monitor_verdict',
            '/monitor_prop14/monitor_verdict' ,
            '/monitor_prop15/monitor_verdict'           
        ]

        # Current status of each topic
        self.status = {t: "unknown" for t in self.topics}

        # Timestamp of the last message received for each topic
        self.last_message_time = {t: 0.0 for t in self.topics}

        # Lock for a thread-safe access (rclpy callbacks vs GUI thread)
        self.lock = threading.Lock()

        # Seconds after which a topic is considered "unknown" (grey)
        self.timeout_seconds = 3.0

        # Create a subscription for each topic
        for topic in self.topics:
            self.create_subscription(String, topic,
                                     lambda msg, t=topic: self.callback(msg, t),
                                     10)

        # Timer to print the state ad each second
        self.timer = self.create_timer(1.0, self.print_status)
        
        self.show_table = True

    def callback(self, msg, topic_name):
        data = msg.data.strip().lower()
        if data not in ["currently_true", "currently_false"]:
            self.get_logger().warn(f"Topic {topic_name}: unexpected value '{msg.data}'")
            return
        with self.lock:
            self.status[topic_name] = data
            self.last_message_time[topic_name] = time.time()

    def check_timeouts(self):
        """Check if some topics are not publishing"""
        current_time = time.time()
        with self.lock:
            for topic in self.topics:
                time_since_last_msg = current_time - self.last_message_time[topic]
                # Se è passato più tempo del timeout e il topic non è già unknown
                if time_since_last_msg > self.timeout_seconds and self.status[topic] != "unknown":
                    if self.last_message_time[topic] > 0:  # Solo se aveva ricevuto almeno un messaggio
                        self.get_logger().warn(f"Topic {topic}: timeout - no message from {time_since_last_msg:.1f}s")
                        self.status[topic] = "unknown"

    def print_property_table(self):
        print("\n" + "="*100)
        print("RECAP TABLE OF MONITORED PROPERTIES")
        print("="*100)
        for prop, formula in self.property_formulas.items():
            print(f"{prop:12s} --> {formula}")
        print("="*100 + "\n")

    def print_status(self):
        # Check the timeouts
        self.check_timeouts()
        
        if self.show_table:
            # Print the table
            self.print_property_table()
        else:
            # Print topic status
            all_true = all(v == "currently_true" for v in self.status.values())
            has_false = any(v == "currently_false" for v in self.status.values())
            has_unknown = any(v == "unknown" for v in self.status.values())
            
            print("\n=== STATUS CHECK ===")
            current_time = time.time()
            
            with self.lock:
                status_copy = dict(self.status)
                last_copy = dict(self.last_message_time)

            for topic, val in status_copy.items():
                if last_copy[topic] > 0:
                    time_since_last = current_time - last_copy[topic]
                    time_info = f"({time_since_last:.1f}s ago)"
                else:
                    time_info = "(never received)"
                print(f"{topic:40s}: {val:15s} {time_info}")
            
            if all_true:
                print("✅  All TRUE")
            elif has_false:
                print("❌  ERROR: at least one topic is currently_false")
            elif has_unknown:
                print("⚠️   WARNING: at least one topic is not publishing (unknown)")
            
            print("====================\n")
        
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
        print("Tkinter not available: starting the gui in a terminal way")
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            print("\n🛑 Keyboard interrupt, closing the GUI ...")
        finally:
            try:
                node.destroy_node()
            except:
                pass
            try:
                rclpy.shutdown()
            except:
                pass
            print("✅ Monitor closed")
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
                m = re.search(r'prop(?:erty)?(?:_)?(poi\d+|10POI\d+|[A-Za-z0-9]+)', topic_clean, re.IGNORECASE)
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
                # Put name and time labels inside the same 'cell' so they stay directly under the rectangle
                name_label = tk.Label(cell, text=name, wraplength=self.square_size+20, justify='center')
                name_label.pack(fill='x')
                # topic property number / short id
                # text = tk.Label(self.frame, text=name, wraplength=self.square_size+20, justify='center')
                # text.grid(row=r*2+1, column=c)
                # last update label
                # time_label = tk.Label(self.frame, text='(never received)', font=('Segoe UI', 8), fg='#555555')
                # time_label.grid(row=r*2+2, column=c)
                time_label = tk.Label(cell, text='(never received)', font=('Segoe UI', 8), fg='#555555')
                time_label.pack(fill='x', pady=(2,4))
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
                    time_label.config(text='(never received)')
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
        print("\n🛑 Keyboard interrupt, closing the GUI ...")
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
        print("✅ Monitor correctly closed")


if __name__ == '__main__':
    main()
