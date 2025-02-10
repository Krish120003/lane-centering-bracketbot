import rerun as rr
import time

# Initialize the Rerun SDK
rr.init("rerun_test_logging")

# Start the Rerun Viewer
rr.connect_tcp("192.168.2.24:9876")

# Log some simple data
for i in range(10):
    rr.set_time_seconds("stable_time", i)
    rr.log("test/data", rr.Points3D([[i, i, i]], colors=[[255, 0, 0]], radii=0.1))
    time.sleep(1)
