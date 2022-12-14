# Loop stress test.
# Sends data: Kvaser -> CAN0 -> CAN1 -> Kvaser
# Too slow, as python is a bottleneck with mutexes. See new rx_test.py instead

from canlib import canlib, Frame
import time
import os
import threading
import multiprocessing


# /*------------------------------------------------------------------*/
# Common:

CHANNEL_NR = 0

# 500kbits, 80% sample
ARBITRATION_PARAMS = canlib.busparams.BusParamsTq(
    tq=80, phase1=63, phase2=16, sjw=4, prescaler=2, prop=0
)
# 4000kbits, 80% sample
DATA_PARAMS = canlib.busparams.BusParamsTq(
    tq=10, phase1=7, phase2=2, sjw=2, prescaler=2, prop=0
)

# Open channels:
tx_channel = canlib.openChannel(channel=CHANNEL_NR, flags=canlib.Open.CAN_FD)
tx_channel.setBusOutputControl(drivertype=canlib.Driver.NORMAL)
tx_channel.set_bus_params_tq(ARBITRATION_PARAMS, DATA_PARAMS)
tx_channel.busOn()

rx_channel = canlib.openChannel(channel=CHANNEL_NR, flags=canlib.Open.CAN_FD)
rx_channel.setBusOutputControl(drivertype=canlib.Driver.NORMAL)
rx_channel.set_bus_params_tq(ARBITRATION_PARAMS, DATA_PARAMS)
rx_channel.busOn()

# /*------------------------------------------------------------------*/
# TX/RX:

sent_frames = []
recv_frames = []
mutex = multiprocessing.Lock()
tx_stop = False
rx_stop = False

can0_frame_count = 0  # Frames sent form here to CAN0
can1_frame_count = 0  # Frames sent from CAN0 to CAN1
unknown_frame_count = 0


def tx_thread_entry():
    global tx_stop
    while not tx_stop:
        # Create frame with random data:
        d = b"\x00" + os.urandom(63)
        frame = Frame(
            id_=0x0,
            dlc=64,
            data=d,
            flags=canlib.MessageFlag.FDF | canlib.MessageFlag.BRS,
        )

        # Send frame
        with mutex:
            sent = False
            while not sent:
                try:
                    tx_channel.write(frame)
                    sent = True
                except Exception:
                    continue
            sent_frames.append(frame)


def rx_thread_entry():
    global rx_stop, can0_frame_count, can1_frame_count, unknown_frame_count
    while not rx_stop:
        with mutex:
            while rx_channel.readStatus() & canlib.Stat.RX_PENDING:
                frame = rx_channel.read()
                if frame.id == 0x200:
                    recv_frames.append(frame)
                elif frame.id == 0x0:
                    can0_frame_count += 1
                elif frame.id == 0x100:
                    can1_frame_count += 1
                else:
                    unknown_frame_count += 1
                    print("ERROR: unknown ID: {}".format(frame.id))


def logic_thread_entry():
    while len(sent_frames) < 10_000:
        with mutex:
            print("tx: {}, rx: {}".format(len(sent_frames), len(recv_frames)))

    global tx_stop
    tx_stop = True
    tx_thread.join()
    time.sleep(0.1)
    global rx_stop
    rx_stop = True
    rx_thread.join()

    global can0_frame_count, can1_frame_count, unknown_frame_count
    print("tx: {}, rx: {}".format(len(sent_frames), len(recv_frames)))
    print(
        "can0: {}, can1: {}, unk: {}".format(
            can0_frame_count, can1_frame_count, unknown_frame_count
        )
    )
    print(
        "tx errs: {};\nrx errs: {}\n".format(
            tx_channel.read_error_counters(), rx_channel.read_error_counters()
        )
    )

    mismatches = 0
    for i in range(len(sent_frames)):
        snt = sent_frames[i].data
        rcv = recv_frames[i].data

        # Sent frames are incremented once at both CANs, so in order to compare to the
        # received frame, we must also make the modification:
        snt[0] += 2

        if snt != rcv:
            # print("frame mismatch! {} != {}".format(snt, rcv))
            # Insert missing frame, otherwise all following frames would mismatch:
            recv_frames.insert(i, snt)
            mismatches += 1

    print("total mismatched frames: {}".format(mismatches))


tx_thread = threading.Thread(target=tx_thread_entry)
rx_thread = threading.Thread(target=rx_thread_entry)
logic_thread = threading.Thread(target=logic_thread_entry)

rx_thread.start()
tx_thread.start()
logic_thread.start()

tx_thread.join()
rx_thread.join()
logic_thread.join()