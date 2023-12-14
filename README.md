# NS-3 simulation scripts for 802.11 with unsaturated traffic
This repository includes the simulator script and patch files used to generate data in following conference paper accepted to *The 38th International Conference on Information Networking (ICOIN) 2024*:
- S. Seytnazarov, D. G. Jeong and W. S. Jeon, "A Simple Performance Model of IEEE 802.11 with Finite Buffer and Load."

## Files
### wifi-11b.h and wifi-11b.cc files
Implement simulation file for 802.11b adhoc/infrastructure network. It accepts command line arguments such as number of nodes, traffic load, seeds, etc.
### script_11b.py file 
Simulation script that uses wifi-11b.cc/.h. It creates number of tasks according your simulation needs such as different number of nodes, traffic loads, seeds, etc. Then it uses multiprocessing: creates multiple processes depending on number of CPU cores you have and they consume created tasks in FIFO order.  
### src.patch file
This patch modifies two files in NS-3.40 src folder.
- ns-3.40/src/applications/model/udp-client.cc file.
  By default UDP application generates packet in fixed intervals. The patch enables the generation of packets in intervals "randomly" generated according to exponential distribution with given mean.
- ns-3.40/src/wifi/model/wifi-remote-station-manager.cc file. Patch enables us to send ACK frames at DsssRate1Mbps. **Important:** It works only in adhoc mode.   
