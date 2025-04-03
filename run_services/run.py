import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import multiprocessing
from thermofischer_interface.web_server import main as server_main
import time
class Manager:
    def __init__(self):
        self.processes = {}

    def add_process(self, process_name, process_function):
        self.processes[process_name] = multiprocessing.Process(target=process_function, daemon=True)
        self.processes[process_name].start()
        
        
    def kill_process(self, process_name):
        self.processes[process_name].terminate()
        self.processes[process_name].join()
        

    def kill_proceses(self):
        for process_name in self.processes:
            self.kill_process(process_name)

    def main(self):
        self.add_process("server", server_main)        

    def close(self):
        self.kill_proceses()

if __name__ == "__main__":
    manager = Manager()
    manager.main()
    try:
        while True:
            time.sleep(10)
            pass
    except KeyboardInterrupt:
        print("Exiting...")
    manager.close()

