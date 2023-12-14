from multiprocessing import Queue
from queue import Empty
import argparse
import warnings
import sys
import os
from multiprocessing import Pool
from multiprocessing import Process, current_process
import time, os
from collections import defaultdict

def worker(tasks: Queue):
    while not tasks.empty():
        n, load, seed = tasks.get_nowait()
        os.system(f'./ns3 run "scratch/wifi-11b --simulationTime={duration} --nStas={n} --load={load} --seed={seed} --infra=0 --verbose=0"')
        print(current_process().name)

if __name__ == "__main__":
    try:
        tasks = Queue()
        duration = 10.0 #sec
        for n in [5, 10, 20]:
            for load in range(1, 21):
                for seed in [1, 2]:
                    tasks.put([n, load/10, seed])
        time.sleep(0.1)
        print(f"{tasks.qsize()} tasks are created")
        print(f"{os.cpu_count()} available cpu cores")
        procs = [Process(target=worker, args=(tasks,)) for i in range(min(tasks.qsize(), os.cpu_count()))]
        # procs = [Process(target=worker, args=(tasks,)) for i in range(1)]
        [p.start() for p in procs]
        [p.join() for p in procs]
    except KeyboardInterrupt:
        for p in procs:
            p.terminate()